"""Offline IMU register, calibration, motor feedback, and fault tests."""

import math
import struct
import threading
import time
import unittest
from unittest.mock import patch

import imu_support as imu
import app as menu
import motors_optimized as motor
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class FakeBus:
    def __init__(self, identity=0x68):
        self.registers = {0x75: identity, 0x3A: 1}
        self.writes = []
        self.burst = list(struct.pack(">7h", 16384, -8192, 4096, 340, 655, -1310, 0))
        self.closed = False

    def read_byte_data(self, address, register):
        return self.registers.get(register, 0)

    def write_byte_data(self, address, register, value):
        self.writes.append((address, register, value))
        self.registers[register] = 0 if register == 0x6B and value == 0x80 else value

    def read_i2c_block_data(self, address, register, count):
        assert register == 0x3B and count == 14
        return self.burst

    def close(self):
        self.closed = True


def sample(sequence=1, timestamp=0, gyro=(0, 0, 0), accel=(0, 0, 1)):
    return imu.Sample(sequence, timestamp, accel, gyro, 25.0)


class RegisterAndMathTests(unittest.TestCase):
    def test_wrong_identity_causes_no_register_writes(self):
        for identity in (0x00, 0xFF, 0x70, 0x71, 0x12):
            with self.subTest(identity=identity):
                bus = FakeBus(identity)
                with self.assertRaises(imu.IMUFault):
                    imu.MPU6050(bus, clock=FakeClock()).initialize()
                self.assertEqual(bus.writes, [])

    def test_initialization_and_signed_burst_conversion(self):
        bus = FakeBus()
        clock = FakeClock()
        sensor = imu.MPU6050(bus, address=0x69, clock=clock)
        sensor.initialize()
        self.assertTrue(all(address == 0x69 for address, _, _ in bus.writes))
        reading = sensor.read_sample()
        self.assertEqual(reading.acceleration, (1, -0.5, 0.25))
        self.assertEqual(reading.gyro, (10, -20, 0))
        self.assertAlmostEqual(reading.temperature, 37.53)
        self.assertEqual(bus.registers[0x1B], 0x08)
        self.assertEqual(bus.registers[0x19], 9)

    def test_configuration_readback_failure_is_rejected(self):
        bus = FakeBus()
        real_read = bus.read_byte_data
        bus.read_byte_data = lambda address, register: 0xFF if register == 0x1B else real_read(address, register)
        with self.assertRaisesRegex(imu.IMUFault, "readback"):
            imu.MPU6050(bus, clock=FakeClock()).initialize()

    def test_no_data_ready_does_not_publish_new_sample(self):
        bus = FakeBus()
        bus.registers[0x3A] = 0
        self.assertIsNone(imu.MPU6050(bus, clock=FakeClock()).read_sample())

    def test_partial_burst_and_saturation_rejected(self):
        for burst in ([0] * 13, list(struct.pack(">7h", 0, 0, 16384, 0, 0, 0, 32767))):
            with self.subTest(burst=burst):
                bus = FakeBus()
                bus.burst = burst
                with self.assertRaises(imu.IMUFault):
                    imu.MPU6050(bus, clock=FakeClock()).read_sample()

    def test_zero_acceleration_and_nan_are_invalid(self):
        for reading in (sample(accel=(0, 0, 0)), sample(gyro=(math.nan, 0, 0))):
            with self.subTest(reading=reading):
                with self.assertRaises(imu.IMUFault):
                    imu.check_sample(reading, 0)

    def test_stale_sample_is_rejected(self):
        with self.assertRaisesRegex(imu.IMUFault, "stale"):
            imu.check_sample(sample(), 0.16)

    def test_sample_delayed_inside_i2c_read_is_stale(self):
        clock = FakeClock()
        bus = FakeBus()

        def slow_burst(*args):
            clock.sleep(0.2)
            return bus.burst

        bus.read_i2c_block_data = slow_burst
        with self.assertRaisesRegex(imu.IMUFault, "stale"):
            imu.MPU6050(bus, clock=clock).read_sample()

    def test_relative_heading_bias_polarity_and_unwrapped_angle(self):
        calibration = imu.Calibration((0, 0, 2), (0, 0, 1), 0, axis=2, sign=-1)
        heading = imu.Heading(calibration, sample(0, 0, gyro=(0, 0, -98)))
        for index in range(1, 201):
            reading = sample(index, index * 0.02, gyro=(0, 0, -98))
            heading.update(reading)
            heading.update(reading)  # duplicate samples must not add angle
        self.assertAlmostEqual(heading.angle, 400)
        self.assertEqual(heading.rate, 100)

    def test_discontinuous_heading_sample_rejected(self):
        calibration = imu.Calibration((0, 0, 0), (0, 0, 1), 0, axis=2)
        heading = imu.Heading(calibration, sample())
        with self.assertRaises(imu.IMUFault):
            heading.update(sample(2, 0.3))

    def test_stationary_bias_calibration(self):
        samples = [sample(i, i * 0.02, gyro=(1, -2, 0.5)) for i in range(100)]
        calibration = imu.calibrate_samples(samples)
        self.assertEqual(calibration.bias, (1, -2, 0.5))
        self.assertIsNone(calibration.axis)

    def test_movement_and_duplicate_data_fail_calibration(self):
        sets = ([sample(i, i * 0.02, gyro=(0, 0, (-1) ** i * 3)) for i in range(100)],
                [sample(1, i * 0.02) for i in range(100)])
        for samples in sets:
            with self.subTest(samples=samples[:2]):
                with self.assertRaises(imu.IMUFault):
                    imu.calibrate_samples(samples)

    def test_reported_138g_readings_calibrate_gyro_and_warn_about_acceleration(self):
        observed = [(-0.23, 6.89, 0.26), (-0.20, 6.84, 0.35), (-0.17, 6.82, 0.31)]
        samples = [sample(i, i * 0.02, gyro=observed[i % 3], accel=(0.19, -0.01, 1.37))
                   for i in range(99)]
        report = imu.calibration_report(samples)
        calibration = report.calibration()
        self.assertAlmostEqual(calibration.bias[1], 6.85)
        self.assertAlmostEqual(report.acceleration_magnitude, 1.3831486, places=5)
        self.assertEqual(calibration.gravity, (0.19, -0.01, 1.37))  # Never rescale readings.
        self.assertIn("1.383 g", report.warnings()[0])
        self.assertIn("NOT corrected", report.warnings()[0])
        self.assertIn("Gyro noise", report.summary())
        imu.verify_axis(calibration, (0, 0, 45))
        heading = imu.Heading(calibration, sample(100, 2, gyro=calibration.bias))
        heading.update(sample(101, 2.02, gyro=calibration.bias))
        self.assertAlmostEqual(heading.angle, 0)

    def test_calibration_reports_which_limit_failed(self):
        cases = (
            (lambda i: sample(i, i * 0.02, gyro=(0, (-1) ** i * 3, 0)), "gyro Y noise"),
            (lambda i: sample(i, i * 0.02, accel=(0, 0, 1 + (-1) ** i * 0.05)), "accel Z noise"),
            (lambda i: sample(i, i * 0.02, gyro=(21, 0, 0)), "gyro X offset"),
        )
        for make_sample, reason in cases:
            with self.subTest(reason=reason):
                with self.assertRaisesRegex(imu.IMUFault, reason):
                    imu.calibrate_samples([make_sample(i) for i in range(100)])

    def test_fixed_tilt_needs_no_acceleration_warning(self):
        report = imu.calibration_report([sample(i, i * 0.02, accel=(0.6, 0, 0.8))
                                         for i in range(100)])
        self.assertEqual(report.warnings(), ())
        self.assertEqual(report.calibration().gravity, (0.6, 0, 0.8))

    def test_calibration_rejects_invalid_or_interrupted_data(self):
        for bad_sample in (sample(50, 1, gyro=(math.nan, 0, 0)),
                           sample(50, 1, accel=(0, 0, 0)),
                           sample(50, 0.5), sample(50, 1.5)):
            with self.subTest(bad_sample=bad_sample):
                samples = [sample(i, i * 0.02) for i in range(100)]
                samples[50] = bad_sample
                with self.assertRaises(imu.IMUFault):
                    imu.calibrate_samples(samples)

    def test_manual_rotation_finds_axis_and_polarity(self):
        for axis in range(3):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    gravity = tuple(1 if i == axis else 0 for i in range(3))
                    calibration = imu.Calibration((0, 0, 0), gravity, 0)
                    angles = [0, 0, 0]
                    angles[axis] = sign * 45
                    imu.verify_axis(calibration, angles)
                    self.assertEqual((calibration.axis, calibration.sign), (axis, sign))

    def test_orientation_rejects_tilt_and_insufficient_rotation(self):
        for angles in ((0, 0, 5), (40, 0, 0), (0, 30, 45)):
            with self.subTest(angles=angles):
                calibration = imu.Calibration((0, 0, 0), (0, 0, 1), 0)
                with self.assertRaises(imu.IMUFault):
                    imu.verify_axis(calibration, angles)

    def test_blocked_i2c_does_not_block_sample_access_or_fault_detection(self):
        clock = FakeClock()
        second_read = threading.Event()
        release = threading.Event()
        bus = FakeBus()

        class BlockingSensor:
            def __init__(self, *args):
                self.calls = 0

            def initialize(self):
                pass

            def read_sample(self):
                self.calls += 1
                if self.calls == 1:
                    return sample()
                second_read.set()
                release.wait(timeout=2)
                return None

        with patch.object(imu, "MPU6050", BlockingSensor):
            stream = imu.IMUStream(bus_factory=lambda number: bus, clock=clock)
            try:
                self.assertTrue(second_read.wait(timeout=1))
                clock.sleep(0.16)
                started = time.monotonic()
                with self.assertRaisesRegex(imu.IMUFault, "stale"):
                    imu.check_sample(stream.latest(), clock.now)
                self.assertLess(time.monotonic() - started, 0.1)
            finally:
                release.set()
                stream.close()
        self.assertTrue(bus.closed)


class DynamicStream:
    def __init__(self, clock, values=None):
        self.clock = clock
        self.values = values or (lambda now: ((0, 0, 0), (0, 0, 1)))
        self.closed = False

    def latest(self):
        gyro, acceleration = self.values(self.clock.now)
        return sample(round(self.clock.now * 100) + 1, self.clock.now, gyro, acceleration)

    def close(self):
        self.closed = True


class IMUMenuTests(unittest.TestCase):
    def make(self, events=(), ready=True):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        stream = DynamicStream(clock)
        session = menu.Session(stream_factory=lambda bus, address: stream)
        session.stream = stream
        if ready:
            session.calibration = imu.Calibration((0, 0, 0), (0, 0, 1), 0, axis=2)
        messages = []
        control = menu.IMUTests(driver, FakeKeys(clock, events), session, clock,
                                lambda text: messages.append((clock.now, text)))
        return control, driver, gpio, clock, stream, session, messages

    def assert_stopped(self, driver, gpio):
        self.assertEqual(driver.duties, [0] * 4)
        self.assertTrue(all(pwm.duty == 0 for pwm in gpio.pwms))

    def test_powered_tests_require_calibration_and_orientation(self):
        for command in ("IF", "ITL", "ITR"):
            for has_calibration in (False, True):
                with self.subTest(command=command, has_calibration=has_calibration):
                    control, driver, gpio, _, _, session, _ = self.make(ready=has_calibration)
                    if session.calibration:
                        session.calibration.axis = None
                    with self.assertRaises(imu.IMUFault):
                        control.run_test(command)
                    self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
                    self.assert_stopped(driver, gpio)

    def test_reidentify_does_not_start_second_worker_while_old_io_is_blocked(self):
        control, driver, gpio, _, stream, session, _ = self.make()
        stream.close = lambda: False
        session.stream_factory = lambda *args: self.fail("A second worker was opened")
        with self.assertRaisesRegex(imu.IMUFault, "still blocked"):
            control.run_test("ID")
        self.assertIs(session.stream, stream)
        self.assertIsNone(session.calibration)
        self.assert_stopped(driver, gpio)

    def test_expired_calibration_prevents_movement(self):
        control, driver, gpio, _, _, session, _ = self.make()
        session.calibration.timestamp = -301
        with self.assertRaisesRegex(imu.IMUFault, "expired"):
            control.run_test("IF")
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_calibration_is_stationary_and_resets_orientation(self):
        control, driver, gpio, _, stream, session, _ = self.make()
        stream.values = lambda now: ((1, -2, 0.5), (0, 0, 1))
        control.run_test("IC")
        self.assertEqual(session.calibration.bias, (1, -2, 0.5))
        self.assertIsNone(session.calibration.axis)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_cancelled_calibration_invalidates_previous_calibration(self):
        control, driver, gpio, _, _, session, _ = self.make([(0.1, "s")])
        with self.assertRaises(motor.StopRequested):
            control.run_test("IC")
        self.assertIsNone(session.calibration)
        self.assert_stopped(driver, gpio)

    def test_calibration_allows_settling_before_collecting_samples(self):
        control, driver, gpio, clock, stream, session, _ = self.make()
        stream.values = lambda now: ((0, 3 if now < 1 else 0, 0), (0, 0, 1))
        control.run_test("IC")
        self.assertEqual(clock.now, menu.CALIBRATION_SETTLE_SECONDS + menu.CALIBRATION_SECONDS)
        self.assertEqual(session.calibration.bias, (0, 0, 0))
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_reported_acceleration_warns_but_does_not_skip_orientation(self):
        control, driver, gpio, _, stream, session, messages = self.make()
        stream.values = lambda now: ((-0.20, 6.85, 0.31), (0.19, -0.01, 1.37))
        control.run_test("IC")
        self.assertIsNone(session.calibration.axis)
        self.assertTrue(any("Acceleration warning" in text for _, text in messages))
        self.assertTrue(any("Gyro calibration passed" in text for _, text in messages))
        with self.assertRaisesRegex(imu.IMUFault, "Run IV"):
            control.run_test("IF")
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_noisy_calibration_reports_reason_and_clears_old_calibration(self):
        control, driver, gpio, _, stream, session, messages = self.make()
        stream.values = lambda now: ((0, (-1) ** round(now * 50) * 3, 0), (0, 0, 1))
        with self.assertRaisesRegex(imu.IMUFault, "gyro Y noise"):
            control.run_test("IC")
        self.assertIsNone(session.calibration)
        self.assertTrue(any("Gyro noise" in text for _, text in messages))
        self.assert_stopped(driver, gpio)

    def test_stop_during_measurement_clears_calibration(self):
        control, driver, gpio, clock, _, session, _ = self.make([(2.1, "s")])
        with self.assertRaises(motor.StopRequested):
            control.run_test("IC")
        self.assertLessEqual(clock.now, 2.12)
        self.assertIsNone(session.calibration)
        self.assert_stopped(driver, gpio)

    def test_stale_data_during_settling_clears_calibration(self):
        control, driver, gpio, clock, stream, session, _ = self.make()
        stream.latest = lambda: sample()
        with self.assertRaisesRegex(imu.IMUFault, "stale"):
            control.run_test("IC")
        self.assertLessEqual(clock.now, 0.18)
        self.assertIsNone(session.calibration)
        self.assert_stopped(driver, gpio)

    def test_orientation_mode_never_powers_motors(self):
        control, driver, gpio, _, stream, session, _ = self.make([(2.0, "g")])
        stream.values = lambda now: ((0, 0, -20), (0, 0, 1))
        control.run_test("IV")
        self.assertEqual((session.calibration.axis, session.calibration.sign), (2, -1))
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_iv_is_kept_through_a_later_ic_while_the_imu_sits_the_same_way(self):
        control, driver, gpio, clock, stream, session, messages = self.make([(2.0, "g")])
        stream.values = lambda now: ((0, 0, -20), (0, 0, 1)) if now < 2.0 else ((0, 0, 0), (0, 0, 1))
        control.run_test("IV")
        self.assertEqual(session.orientation, (2, -1, True))
        control.run_test("IC")  # the robot rests still; only the bias is re-measured
        self.assertEqual((session.calibration.axis, session.calibration.sign), (2, -1))
        self.assertEqual(menu.imu_status(session, clock.now), "Ready")
        self.assertIn("Orientation kept from the last IV: sensor Z, sign -1. Ready.", messages[-1][1])
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_a_flipped_or_tilted_imu_drops_the_kept_iv(self):
        for gravity in ((0, 0, -1), (0.6, 0, 0.8)):  # upside down; tilted past the vertical limit
            with self.subTest(gravity=gravity):
                control, driver, gpio, clock, stream, session, messages = self.make()
                session.orientation = (2, 1, True)
                stream.values = lambda now, gravity=gravity: ((0, 0, 0), gravity)
                control.run_test("IC")
                self.assertIsNone(session.calibration.axis)
                self.assertIsNone(session.orientation)
                self.assertEqual(menu.imu_status(session, clock.now), "Needs IV")
                self.assertIn("that IV is dropped. Run IV again.", messages[-1][1])

    def test_the_kept_orientation_also_works_on_another_axis(self):
        control, driver, gpio, clock, stream, session, _ = self.make()
        session.orientation = (0, 1, False)  # IMU on its edge: yaw on sensor X, gravity along -X
        stream.values = lambda now: ((0, 0, 0), (-1, 0, 0))
        control.run_test("IC")
        self.assertEqual((session.calibration.axis, session.calibration.sign), (0, 1))

    def test_a_failed_ic_keeps_the_iv_for_the_next_ic_but_a_cancelled_iv_forgets_it(self):
        control, driver, gpio, clock, stream, session, _ = self.make([(0.1, "s")])
        session.orientation = (2, 1, True)
        with self.assertRaises(motor.StopRequested):
            control.run_test("IC")  # cancelled: no calibration, the mounting is unchanged
        self.assertEqual(session.orientation, (2, 1, True))
        control.run_test("IC")
        self.assertEqual(session.calibration.axis, 2)
        control.keys = FakeKeys(clock, [(clock.now + 0.1, "s")])
        with self.assertRaises(motor.StopRequested):
            control.run_test("IV")  # an IV that does not pass leaves nothing to keep
        self.assertIsNone(session.orientation)
        control.keys = FakeKeys(clock)
        control.run_test("IC")
        self.assertIsNone(session.calibration.axis)

    def test_closing_the_session_forgets_the_iv(self):
        _, _, _, _, _, session, _ = self.make()
        session.orientation = (2, 1, True)
        self.assertTrue(session.close())
        self.assertIsNone(session.orientation)

    def test_live_readings_stop_without_powering_motors(self):
        control, driver, gpio, _, _, _, messages = self.make([(0.25, "s")], ready=False)
        with self.assertRaises(motor.StopRequested):
            control.run_test("IMU")
        self.assertTrue(any("gyro XYZ" in text for _, text in messages))
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))
        self.assert_stopped(driver, gpio)

    def test_stop_key_during_each_powered_imu_test(self):
        for command in ("IF", "ITL", "ITR"):
            with self.subTest(command=command):
                control, driver, gpio, clock, _, _, _ = self.make([(0.05, "s")])
                with self.assertRaises(motor.StopRequested):
                    control.run_test(command)
                self.assertLessEqual(clock.now, 0.07)
                self.assert_stopped(driver, gpio)

    def test_stale_stream_stops_powered_test(self):
        control, driver, gpio, clock, stream, _, _ = self.make()
        stream.latest = lambda: sample()
        with self.assertRaisesRegex(imu.IMUFault, "stale"):
            control.run_test("IF")
        self.assertLessEqual(clock.now, 0.18)
        self.assert_stopped(driver, gpio)

    def test_front_line_stops_imu_forward_during_ramp(self):
        control, driver, gpio, clock, _, _, _ = self.make()
        gpio.readings = lambda now: (1, 1, 0 if now >= 0.2 else 1)
        control.run_test("IF")
        self.assertEqual(clock.now, 0.2)
        self.assert_stopped(driver, gpio)

    def test_forward_test_is_bounded_and_duty_limited(self):
        control, driver, gpio, clock, _, _, _ = self.make()
        control.run_test("IF")
        self.assertEqual(clock.now, menu.IMU_DRIVE_SECONDS)
        self.assertLessEqual(max(d for _, _, d in gpio.pwm_writes), menu.IMU_DRIVE_DUTY)
        self.assert_stopped(driver, gpio)

    def test_heading_correction_sign_and_limits(self):
        straight = menu.forward_targets(0, 0)
        left_error = menu.forward_targets(5, 0)
        right_error = menu.forward_targets(-5, 0)
        self.assertLess(left_error[1], straight[1])  # steer right by slowing right
        self.assertLess(right_error[0], straight[0])
        for value in menu.forward_targets(1000, 1000):
            self.assertTrue(0 <= value <= menu.IMU_DRIVE_DUTY)

    def test_large_heading_error_stops_forward(self):
        control, driver, gpio, _, stream, _, _ = self.make()
        stream.values = lambda now: ((0, 0, 0 if now == 0 else 100), (0, 0, 1))
        with self.assertRaisesRegex(imu.IMUFault, "departed"):
            control.run_test("IF")
        self.assert_stopped(driver, gpio)

    def test_no_turn_progress_stops(self):
        control, driver, gpio, clock, _, _, _ = self.make()
        with self.assertRaisesRegex(imu.IMUFault, "progress"):
            control.run_test("ITL")
        self.assertLessEqual(clock.now, 1.02)
        self.assert_stopped(driver, gpio)

    def test_wrong_turn_direction_stops(self):
        control, driver, gpio, _, stream, _, _ = self.make()
        stream.values = lambda now: ((0, 0, 0 if now == 0 else -40), (0, 0, 1))
        with self.assertRaisesRegex(imu.IMUFault, "wrong measured direction"):
            control.run_test("ITL")
        self.assert_stopped(driver, gpio)

    def test_turn_timeout_limits_slow_movement(self):
        control, driver, gpio, clock, stream, _, _ = self.make()
        stream.values = lambda now: ((0, 0, 0 if now == 0 else 2), (0, 0, 1))
        with self.assertRaisesRegex(imu.IMUFault, "timed out"):
            control.run_test("ITL")
        self.assertLessEqual(clock.now, menu.TURN_TIMEOUT + 0.02)
        self.assert_stopped(driver, gpio)

    def test_left_and_right_turns_follow_simulated_gyro_angle(self):
        for command in ("ITL", "ITR"):
            for sensor_sign in (1, -1):
                with self.subTest(command=command, sensor_sign=sensor_sign):
                    control, driver, gpio, clock, stream, session, messages = self.make()
                    session.calibration.sign = sensor_sign
                    plant = {"rate": 0.0, "angle": 0.0}
                    original_sleep = clock.sleep

                    def sleep(duration):
                        mean_duty = sum(driver.duties) / 4
                        if driver.pattern == motor.TURN_LEFT:
                            plant["rate"] = mean_duty * 3
                        elif driver.pattern == motor.TURN_RIGHT:
                            plant["rate"] = -mean_duty * 3
                        else:
                            plant["rate"] = 0
                        plant["angle"] += plant["rate"] * duration
                        original_sleep(duration)

                    clock.sleep = sleep
                    stream.values = lambda now: ((0, 0, plant["rate"] * sensor_sign), (0, 0, 1))
                    control.run_test(command)
                    self.assertLess(abs(abs(plant["angle"]) - 90), 5)
                    self.assertTrue(any("Turn stopped:" in text for _, text in messages))
                    self.assertLess(clock.now, menu.TURN_TIMEOUT)
                    self.assert_stopped(driver, gpio)

    def test_imu_failure_stops_and_does_not_fall_back_to_timed_turn(self):
        control, driver, gpio, clock, stream, _, _ = self.make()

        def failing_read():
            if clock.now >= 0.1:
                raise imu.IMUFault("Injected I2C error")
            return sample(round(clock.now * 100) + 1, clock.now)

        stream.latest = failing_read
        with self.assertRaisesRegex(imu.IMUFault, "I2C error"):
            control.run_test("ITR")
        self.assertEqual(clock.now, 0.1)
        self.assert_stopped(driver, gpio)


if __name__ == "__main__":
    unittest.main(verbosity=2)
