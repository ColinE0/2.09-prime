"""Offline INA219 register, battery display, and terminal responsiveness tests."""

import contextlib
import io
import math
from pathlib import Path
import sys
import threading
import time
import types
import unittest
import tempfile
from unittest.mock import patch

import battery_support as battery
import app as menu
import motors_optimized as motor
from test_motors_optimized import FakeClock, FakeGPIO, FakeKeys


class FakeBatteryBus:
    def __init__(self, clock, voltage=7.68, shunt_mv=1.0):
        self.clock = clock
        self.registers = {0: 0x399F, 1: round(shunt_mv * 100) & 0xFFFF,
                          2: round(voltage / 0.004) << 3}
        self.ready_at = math.inf
        self.never_ready = self.closed = False
        self.writes = []

    def write_i2c_block_data(self, address, register, raw):
        value = (raw[0] << 8) | raw[1]
        self.registers[register] = value
        self.writes.append((address, register, list(raw)))
        if register == 0 and value & 7 == 3:
            self.ready_at = self.clock.now + 0.02

    def read_i2c_block_data(self, address, register, count):
        assert count == 2
        value = self.registers[register]
        if register == 2:
            value &= ~2
            if not self.never_ready and self.clock.now >= self.ready_at:
                value |= 2
        return [value >> 8, value & 0xFF]

    def close(self):
        self.closed = True


def reading(timestamp=0, voltage=7.68, current=0.1):
    shunt = current * 0.01
    return battery.BatterySample(timestamp, voltage - shunt, shunt, current)


def profile():
    return battery.load_battery_profile(Path(__file__).with_name("battery_profile.json"))


class INA219Tests(unittest.TestCase):
    def test_big_endian_signed_conversion_and_fresh_paired_samples(self):
        for current in (0.1, -0.5):
            with self.subTest(current=current):
                clock = FakeClock()
                bus = FakeBatteryBus(clock, voltage=7.68, shunt_mv=current * 10)
                sensor = battery.INA219(bus, clock=clock)
                sensor.initialize()
                sample = sensor.read_sample()
                self.assertEqual(bus.writes[0], (0x40, 0, [0x2E, 0x60]))
                self.assertEqual(bus.writes[1], (0x40, 0, [0x2E, 0x63]))
                self.assertAlmostEqual(sample.bus_voltage, 7.68)
                self.assertAlmostEqual(sample.current, current)
                self.assertAlmostEqual(sample.input_voltage, 7.68 + current * 0.01)
                self.assertAlmostEqual(sample.power, sample.input_voltage * current)
                self.assertEqual(clock.now, 0.02)
                sample = sensor.read_sample()
                self.assertEqual(clock.now, 0.04)  # A second fresh conversion, not cached data.
                self.assertEqual(sample.timestamp, 0.02)

    def test_r100_shunt_and_alternate_address(self):
        clock = FakeClock()
        bus = FakeBatteryBus(clock, shunt_mv=10)
        sensor = battery.INA219(bus, address=0x41, shunt_ohms=0.1, clock=clock)
        sensor.initialize()
        self.assertAlmostEqual(sensor.read_sample().current, 0.1)
        self.assertTrue(all(address == 0x41 for address, _, _ in bus.writes))

    def test_invalid_shunt_or_address_rejected(self):
        for shunt in (0, -1, math.nan, math.inf):
            with self.subTest(shunt=shunt), self.assertRaises(ValueError):
                battery.INA219(None, shunt_ohms=shunt)
        with self.assertRaises(ValueError):
            battery.INA219(None, address=0x68)

    def test_bad_initial_configuration_causes_no_writes(self):
        bus = FakeBatteryBus(FakeClock())
        bus.registers[0] = 0xFFFF
        with self.assertRaisesRegex(battery.BatteryFault, "Unexpected"):
            battery.INA219(bus).initialize()
        self.assertEqual(bus.writes, [])

    def test_configuration_readback_mismatch(self):
        bus = FakeBatteryBus(FakeClock())
        bus.write_i2c_block_data = lambda *args: None
        with self.assertRaisesRegex(battery.BatteryFault, "readback"):
            battery.INA219(bus).initialize()

    def test_missing_sensor_and_partial_reads(self):
        bus = FakeBatteryBus(FakeClock())
        bus.read_i2c_block_data = lambda *args: [0]
        with self.assertRaisesRegex(battery.BatteryFault, "Incomplete"):
            battery.INA219(bus).initialize()
        bus.read_i2c_block_data = lambda *args: (_ for _ in ()).throw(OSError("No device"))
        with self.assertRaises(OSError):
            battery.INA219(bus).initialize()

    def test_conversion_timeout_is_bounded(self):
        clock = FakeClock()
        bus = FakeBatteryBus(clock)
        bus.never_ready = True
        sensor = battery.INA219(bus, clock=clock)
        sensor.initialize()
        with self.assertRaisesRegex(battery.BatteryFault, "timed out"):
            sensor.read_sample()
        self.assertEqual(clock.now, 0.25)

    def test_saturation_and_out_of_range_voltage(self):
        for voltage, shunt_mv in ((7.4, 80), (7.4, -80), (26.1, 0)):
            with self.subTest(voltage=voltage, shunt_mv=shunt_mv):
                clock = FakeClock()
                sensor = battery.INA219(FakeBatteryBus(clock, voltage, shunt_mv), clock=clock)
                sensor.initialize()
                with self.assertRaises(battery.BatteryFault):
                    sensor.read_sample()

    def test_calibrated_math_overflow_does_not_corrupt_raw_voltage_current(self):
        clock = FakeClock()
        bus = FakeBatteryBus(clock)
        bus.registers[2] |= 1  # Chip's current/power math is not used by this driver.
        sensor = battery.INA219(bus, clock=clock)
        sensor.initialize()
        self.assertAlmostEqual(sensor.read_sample().current, 0.1)

    def test_stale_and_invalid_samples(self):
        for sample, now in ((reading(), 0.76), (reading(), -0.1),
                            (reading(current=math.nan), 0)):
            with self.subTest(sample=sample), self.assertRaises(battery.BatteryFault):
                battery.check_battery_sample(sample, now)

    def test_blocked_worker_does_not_block_latest_or_allow_duplicate_owner(self):
        clock = FakeClock()
        entered = threading.Event()
        release = threading.Event()
        bus = FakeBatteryBus(clock)

        class BlockingSensor:
            def __init__(self, *args):
                pass

            def initialize(self):
                pass

            def read_sample(self):
                entered.set()
                release.wait(timeout=2)
                return reading()

        with patch.object(battery, "INA219", BlockingSensor):
            stream = battery.BatteryStream(bus_factory=lambda number: bus, clock=clock)
            try:
                self.assertTrue(entered.wait(timeout=1))
                started = time.monotonic()
                self.assertIsNone(stream.latest())
                self.assertLess(time.monotonic() - started, 0.1)
                self.assertFalse(stream.close())
            finally:
                release.set()
                stream.close()
        self.assertTrue(bus.closed)


class ChargeDisplayTests(unittest.TestCase):
    def test_bundled_profile_matches_user_battery_and_interpolates(self):
        pack = profile()
        self.assertEqual(pack.cells, 2)
        self.assertIn("2000", pack.name)
        self.assertAlmostEqual(pack.estimate(7.70), 50)
        self.assertAlmostEqual(pack.estimate(7.65), 45)
        self.assertEqual(pack.estimate(6.0), 0)
        self.assertEqual(pack.estimate(8.5), 100)

    def test_voltage_and_coarse_charge_bar_display(self):
        text = battery.battery_status(reading(voltage=7.70), profile())
        self.assertIn("7.70 V", text)
        self.assertIn("+0.100 A", text)
        self.assertIn("~50% [#####-----]", text)

    def test_low_battery_and_disconnected_input_are_distinct(self):
        self.assertIn("LOW BATTERY", battery.battery_status(reading(voltage=6.6), profile()))
        self.assertIn("does not match", battery.battery_status(reading(voltage=0), profile()))
        self.assertNotIn("~0%", battery.battery_status(reading(voltage=0), profile()))

    def test_unknown_profile_high_load_and_reverse_flow_never_show_percentage(self):
        for sample, pack, reason in ((reading(), None, "not configured"),
                                     (reading(current=1), profile(), "load too high"),
                                     (reading(current=-0.1), profile(), "reverse current")):
            with self.subTest(reason=reason):
                text = battery.battery_status(sample, pack)
                self.assertIn(reason, text)
                self.assertNotIn("%", text)

    def test_invalid_profile_rejected(self):
        for points in (((3.3, 0), (3.3, 100)), ((3.3, 0), (4.2, 50)),
                       ((3.3, 0), (math.nan, 100)), ((3.3, 0), (4.2, -1))):
            with self.subTest(points=points), self.assertRaises(ValueError):
                battery.BatteryProfile("invalid", 2, points, 3.3, 0.2)


class BatteryMenuTests(unittest.TestCase):
    def make(self, events=()):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        driver = motor.MotorDriver(gpio)
        driver.initialize()
        messages = []
        control = motor.Controller(driver, FakeKeys(clock, events), clock, messages.append)
        stream = types.SimpleNamespace(latest=lambda: reading(clock.now), close=lambda: True)
        session = battery.BatterySession(profile=profile(), stream_factory=lambda *args: stream)
        return control, session, stream, driver, gpio, clock, messages

    def assert_off(self, driver, gpio):
        self.assertEqual(driver.duties, [0] * 4)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))

    def test_live_screen_and_stop_key_never_power_motors(self):
        control, session, _, driver, gpio, clock, messages = self.make([(0.1, "s")])
        with self.assertRaises(motor.StopRequested):
            session.readings(control)
        self.assertEqual(clock.now, 0.1)
        self.assertTrue(any("Charge estimate" in text for text in messages))
        self.assertIsNone(session.stream)
        self.assert_off(driver, gpio)

    def test_stop_key_works_before_first_reading(self):
        control, session, stream, driver, gpio, clock, _ = self.make([(0.04, "s")])
        stream.latest = lambda: None
        with self.assertRaises(motor.StopRequested):
            session.readings(control)
        self.assertEqual(clock.now, 0.04)
        self.assert_off(driver, gpio)

    def test_stale_reading_returns_to_menu_without_reprinting_charge(self):
        control, session, stream, driver, gpio, clock, messages = self.make()
        stream.latest = lambda: reading()
        with self.assertRaisesRegex(battery.BatteryFault, "stale"):
            session.readings(control)
        self.assertLessEqual(clock.now, 0.78)
        self.assertEqual(sum("Charge estimate" in text for text in messages), 1)
        self.assert_off(driver, gpio)

    def test_no_readings_times_out_and_i2c_fault_propagates(self):
        for fault in (False, True):
            with self.subTest(fault=fault):
                control, session, stream, driver, gpio, clock, _ = self.make()
                stream.latest = (lambda: (_ for _ in ()).throw(battery.BatteryFault("I2C error"))) if fault else lambda: None
                with self.assertRaises(battery.BatteryFault):
                    session.readings(control)
                self.assertLessEqual(clock.now, 2.0)
                self.assertIsNone(session.stream)
                self.assert_off(driver, gpio)

    def test_blocked_old_worker_cannot_be_replaced(self):
        control, session, stream, driver, gpio, _, _ = self.make()
        session.stream = stream
        stream.close = lambda: False
        session.stream_factory = lambda *args: self.fail("Opened duplicate INA219 worker")
        with self.assertRaisesRegex(battery.BatteryFault, "still blocked"):
            session.readings(control)
        self.assertIs(session.stream, stream)
        self.assert_off(driver, gpio)

    def test_menu_dispatches_battery_and_returns_after_sensor_error(self):
        clock = FakeClock()
        gpio = FakeGPIO(clock)
        rpi = types.ModuleType("RPi")
        rpi.GPIO = gpio
        called = []
        fake_session = types.SimpleNamespace(close=lambda: called.append("close"))

        def fail_readings(control):
            called.append("BAT")
            raise battery.BatteryFault("No sensor attached")

        fake_session.readings = fail_readings
        choices = iter(["BAT", "Q"])
        output = io.StringIO()
        with patch.dict(sys.modules, {"RPi": rpi, "RPi.GPIO": gpio}), \
             patch.object(menu.sys.stdin, "isatty", return_value=True), \
             patch.object(menu, "BatterySession", return_value=fake_session), \
             patch.object(motor, "prompt_line", side_effect=lambda text: next(choices)), \
             patch.object(motor, "Keyboard", side_effect=lambda: contextlib.nullcontext(FakeKeys(clock))), \
             contextlib.redirect_stdout(output):
            with tempfile.TemporaryDirectory() as folder:
                self.assertEqual(menu.main(["--bot", "500", "--settings", str(Path(folder) / "settings.json")]), 0)
        self.assertEqual(called, ["BAT", "close"])
        self.assertIn("No sensor attached", output.getvalue())
        self.assertTrue(gpio.cleaned)
        self.assertTrue(all(duty == 0 for _, _, duty in gpio.pwm_writes))


if __name__ == "__main__":
    unittest.main(verbosity=2)
