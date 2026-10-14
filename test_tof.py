"""Off-Pi ToF policy, coherent I2C results, and bounded process lifecycle."""

import math
from queue import Queue
import sys
import time
import types
import unittest
from unittest.mock import patch

import tof_support as tof
from test_motors_optimized import FakeClock


class GateTests(unittest.TestCase):
    def setUp(self):
        self.value = None
        self.sequence = 0
        self.stream = types.SimpleNamespace(latest=lambda: self.value)
        self.gate = tof.ToFGate(self.stream, log=lambda text: None)

    def feed(self, stamp, mm=600, status=11):
        self.sequence += 1
        self.value = tof.ToFSample(self.sequence, stamp, mm, status)
        return self.gate.check(stamp)

    def clear(self, start=0):
        for index in range(16):
            result = self.feed(round(start + index * 0.05, 6))
        return result

    def test_initial_clear_requires_continuous_fresh_readings_for_075_seconds(self):
        for index in range(15):
            self.assertEqual(self.feed(index * 0.05)[0], "hold")
        self.assertEqual(self.feed(0.75)[:2], ("drive", 1.0))

    def test_duplicates_do_not_confirm_clear_and_stale_duplicates_fail(self):
        self.feed(0)
        for now in (0.05, 0.10, 0.2, 0.29):
            self.assertEqual(self.gate.check(now)[0], "hold")
        with self.assertRaisesRegex(tof.ToFFault, "stale"):
            self.gate.check(0.31)

    def test_close_object_brakes_on_first_sample_and_hysteresis_requires_above_90(self):
        self.clear()
        self.assertEqual(self.feed(0.80, 80)[:2], ("hold", 0.0))
        for index in range(1, 18):
            self.assertEqual(self.feed(0.80 + index * 0.05, 90)[0], "hold")
        self.assertEqual(self.clear(start=1.70)[:2], ("drive", 1.0))

    def test_slowdown_scales_by_distance_without_exceeding_original_duty(self):
        self.clear()
        for now, mm, wanted in ((0.80, 100, 1.0), (0.85, 90, 0.7),
                                (0.90, 82, 0.46), (0.95, 800, 1.0)):
            action, scale, _ = self.feed(now, mm)
            self.assertEqual(action, "drive")
            self.assertAlmostEqual(scale, wanted)

    def test_obstacle_or_long_measurement_gap_restarts_clear_window(self):
        self.feed(0)
        self.feed(0.05)
        self.feed(0.25)  # 200 ms gap; individually fresh but not a clear streak.
        for index in range(1, 15):
            self.assertEqual(self.feed(0.25 + index * 0.05)[0], "hold")
        self.assertEqual(self.feed(1.0)[0], "drive")
        self.feed(1.05, 20)
        self.feed(1.10, 600)
        self.feed(1.15, 600)
        self.feed(1.20, 89)
        for index in range(1, 16):
            self.assertEqual(self.feed(1.20 + index * 0.05)[0], "hold")
        self.assertEqual(self.feed(2.0)[0], "drive")

    def test_unknown_statuses_and_zero_distance_never_mean_clear(self):
        for mm, status in ((0, 11), (500, 0), (500, 15), (500, 1), (500, 5), (500, 7), (8190, 12)):
            with self.subTest(mm=mm, status=status):
                self.setUp()
                self.clear()
                self.assertEqual(self.feed(0.80, mm, status)[:2], ("hold", 0.0))

    def test_nothing_in_range_reads_clear_down_an_open_lane(self):
        for mm, status in ((8190, 11), (2001, 11), (8190, 4), (8191, 6), (8190, 9), (8190, 14)):
            with self.subTest(mm=mm, status=status):
                self.setUp()
                self.clear()
                self.assertEqual(self.feed(0.80, mm, status)[:2], ("drive", 1.0))

    def test_open_space_from_the_start_clears_and_never_ends_fc(self):
        results = [self.feed(index * 0.05, 8190, 4)[0] for index in range(60)]
        self.assertEqual(results[:15], ["hold"] * 15)  # the usual 0.75 s clear window
        self.assertEqual(set(results[15:]), {"drive"})

    def test_target_inside_minimum_range_holds(self):
        for mm, status in ((20, 8), (0, 10)):
            with self.subTest(mm=mm, status=status):
                self.setUp()
                self.clear()
                action, scale, reason = self.feed(0.80, mm, status)
                self.assertEqual((action, scale), ("hold", 0.0))
                self.assertIn("minimum range", reason)

    def test_unknown_reading_holds_then_ends_after_one_second(self):
        for index in range(20):
            self.assertEqual(self.feed(index * 0.05, 500, 0)[0], "hold")
        with self.assertRaisesRegex(tof.ToFFault, "no usable reading"):
            self.feed(1.0, 500, 0)

    def test_transient_invalid_result_requires_fresh_clear_streak_to_resume(self):
        self.clear()
        self.feed(0.8, 500, 0)
        for index in range(15):
            self.assertEqual(self.feed(0.85 + index * 0.05)[0], "hold")
        self.assertEqual(self.feed(1.60)[0], "drive")

    def test_motors_off_gate_can_report_persistent_invalid_without_ending(self):
        self.gate = tof.ToFGate(self.stream, log=lambda text: None, invalid_timeout=None)
        for index in range(50):
            self.assertEqual(self.feed(index * 0.05, 500, 0)[0], "hold")

    def test_out_of_order_and_mutated_duplicate_fail(self):
        for replacement in (tof.ToFSample(1, 0.05, 500, 11), tof.ToFSample(2, 0.05, 500, 11),
                            tof.ToFSample(2, 0.10, 501, 11), tof.ToFSample(3, 0.10, 500, 11)):
            with self.subTest(replacement=replacement):
                self.setUp()
                self.feed(0.05, 500)
                self.feed(0.10, 500)
                self.value = replacement
                with self.assertRaises(tof.ToFFault):
                    self.gate.check(0.12)

    def test_bad_results_are_rejected_before_policy(self):
        for value in (None, tof.ToFSample(0, 0, 500, 11), tof.ToFSample(1, math.nan, 500, 11),
                      tof.ToFSample(1, "bad", 500, 11), tof.ToFSample(1, None, 500, 11),
                      tof.ToFSample(1, 1, 500, 11), tof.ToFSample(1, 0, -1, 11),
                      tof.ToFSample(1, 0, 500.0, 11), tof.ToFSample(1, 0, 500, 16)):
            with self.subTest(value=value), self.assertRaises(tof.ToFFault):
                tof.check_tof_sample(value, 0)


class FakeI2CDevice:
    def __init__(self, sensor):
        self.sensor = sensor
        self.locked = False

    def __enter__(self):
        self.locked = True
        return self

    def __exit__(self, *args):
        self.locked = False

    def write(self, data):
        assert self.locked and bytes(data) == b"\x14"

    def readinto(self, data):
        assert self.locked and len(data) == 12
        data[0] = self.sensor.status << 3
        data[10], data[11] = divmod(self.sensor.mm, 256)


class FakeSensor:
    def __init__(self, clock):
        self.clock = clock
        self.mm, self.status = 612, 11
        self.ready_at = float("inf")
        self.ready_delay = 0.03
        self._data_ready = True  # An old cached ready value must not be used.
        self._device = FakeI2CDevice(self)
        self.writes = []
        self.shots = 0

    def _write_u8(self, register, value):
        self.writes.append((register, value))

    def do_range_measurement(self):
        self.shots += 1
        self.ready_at = self.clock.now + self.ready_delay

    @property
    def data_ready(self):
        if self.clock.now >= self.ready_at:
            self._data_ready = True
        return self._data_ready


class DeviceTests(unittest.TestCase):
    def make(self):
        clock = FakeClock()
        sensor = FakeSensor(clock)
        device = tof.VL53L0XDevice.__new__(tof.VL53L0XDevice)
        device.sensor, device.i2c = sensor, None
        done = types.SimpleNamespace(is_set=lambda: False, wait=clock.sleep)
        return device, sensor, done, clock

    def test_new_single_shot_coherent_status_and_distance_and_acknowledgement(self):
        device, sensor, done, clock = self.make()
        started, distance, status = device.read(done, clock)
        self.assertEqual((started, distance, status), (0, 612, 11))
        self.assertGreaterEqual(clock.now, 0.03)
        self.assertEqual(sensor.shots, 1)
        self.assertEqual(sensor.writes, [(0x0B, 1), (0x0B, 1)])
        self.assertFalse(sensor._data_ready)
        sensor.mm, sensor.status = 8190, 4
        started, distance, status = device.read(done, clock)
        self.assertEqual((distance, status), (8190, 4))  # Never clip invalid to clear.
        self.assertEqual(sensor.shots, 2)

    def test_not_ready_times_out_instead_of_reusing_last_distance(self):
        device, sensor, done, clock = self.make()
        sensor.ready_delay = 10
        with self.assertRaisesRegex(tof.ToFFault, "timed out"):
            device.read(done, clock)
        self.assertLessEqual(clock.now, 0.205)

    def test_cancellation_during_measurement_returns_without_publishing(self):
        device, sensor, done, clock = self.make()
        done.is_set = lambda: clock.now >= 0.01
        self.assertIsNone(device.read(done, clock))
        self.assertLess(clock.now, 0.03)

    def test_bus_read_failure_propagates(self):
        device, sensor, done, clock = self.make()
        sensor._device.readinto = lambda data: (_ for _ in ()).throw(OSError("I2C failed"))
        with self.assertRaises(OSError):
            device.read(done, clock)

    def test_constructor_uses_named_bus_timeout_and_discards_prior_mode_result(self):
        calls = []
        bus = types.SimpleNamespace(deinit=lambda: calls.append("deinit"))
        sensor = types.SimpleNamespace(stop_continuous=lambda: calls.append("stop"),
                                       read_range=lambda: calls.append("discard"))
        def sensor_factory(i2c, **kwargs):
            self.assertIs(i2c, bus)
            self.assertEqual(kwargs, {"address": 0x29, "io_timeout_s": 0.2})
            return sensor
        with patch.dict(sys.modules, {
            "adafruit_extended_bus": types.SimpleNamespace(ExtendedI2C=lambda index: calls.append(index) or bus),
            "adafruit_vl53l0x": types.SimpleNamespace(VL53L0X=sensor_factory),
        }):
            device = tof.VL53L0XDevice(1, 0x29)
            self.assertEqual(sensor.measurement_timing_budget, 33000)
            device.close()
            device.close()
        self.assertEqual(calls, [1, "stop", "discard", "deinit"])

    def test_constructor_failure_closes_open_bus(self):
        closed = []
        with patch.dict(sys.modules, {
            "adafruit_extended_bus": types.SimpleNamespace(ExtendedI2C=lambda index: types.SimpleNamespace(deinit=lambda: closed.append(True))),
            "adafruit_vl53l0x": types.SimpleNamespace(VL53L0X=lambda *args, **kwargs: (_ for _ in ()).throw(OSError("no device"))),
        }), self.assertRaises(OSError):
            tof.VL53L0XDevice(1, 0x29)
        self.assertEqual(closed, [True])

    def test_worker_preserves_measurement_time_and_invalid_status(self):
        queue = Queue()
        stopped, closed = [], []
        device = types.SimpleNamespace(read=lambda done: (5.0, 8190, 4), close=lambda: closed.append(True))
        done = types.SimpleNamespace(is_set=lambda: bool(stopped), wait=lambda delay: stopped.append(True))
        with patch.object(tof, "VL53L0XDevice", return_value=device), \
             patch.object(tof.time, "monotonic", return_value=5.04):
            tof.tof_worker(1, 0x29, queue, done)
        result = queue.get_nowait()
        self.assertEqual(result, tof.ToFSample(1, 5.0, 8190, 4))
        self.assertFalse(result.usable)
        self.assertEqual(closed, [True])


def stuck_worker(bus, address, queue, done):
    queue.put(tof.ToFSample(1, time.monotonic(), 500, 11))
    time.sleep(30)


def failed_worker(bus, address, queue, done):
    queue.put(("error", "Injected I2C failure"))
    done.wait(5)


class WorkerTests(unittest.TestCase):
    def wait_result(self, stream):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            result = stream.latest()
            if result is not None:
                return result
            time.sleep(0.02)
        self.fail("Test worker did not publish")

    def test_stuck_sensor_process_can_be_terminated_promptly(self):
        stream = tof.ToFStream(worker=stuck_worker)
        try:
            self.assertEqual(self.wait_result(stream).distance_mm, 500)
            started = time.monotonic()
            self.assertTrue(stream.close())
            self.assertLess(time.monotonic() - started, 2)
            self.assertFalse(stream.process.is_alive())
            self.assertTrue(stream.forced_shutdown)
            self.assertTrue(stream.close())
        finally:
            stream.close()

    def test_worker_error_is_latched_and_cleanup_is_bounded(self):
        stream = tof.ToFStream(worker=failed_worker)
        try:
            with self.assertRaisesRegex(tof.ToFFault, "Injected I2C failure"):
                self.wait_result(stream)
            with self.assertRaisesRegex(tof.ToFFault, "Injected I2C failure"):
                stream.latest()
        finally:
            self.assertTrue(stream.close())


if __name__ == "__main__":
    unittest.main(verbosity=2)
