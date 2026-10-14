"""Reproduce SET-to-wheel output and INA219 recovery/ownership failures."""
import contextlib
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

import app
import battery_support as battery
import motors_optimized as motor
import power_guard
import robot_profiles
import robot_settings as settings
import terminal_ui
from single_motor import SingleMotorTests
from test_app import AppHarness
from test_bot_switching import SwitchingHarness
from test_motors_optimized import FakeClock, FakeKeys
from test_battery import FakeBatteryBus, reading
from test_v9_1 import ScriptedStream, FakeMotors, make_guard


class ReversalApplicationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))

    def test_real_editor_flips_each_wheel_on_both_bots_before_next_wheel_test(self):
        for bot in ("500", "100"):
            for index, wheel in enumerate(("FL", "FR", "BL", "BR")):
                with self.subTest(bot=bot, wheel=wheel):
                    path = self.root / f"{bot}-{wheel}.json"
                    settings.save_settings(path, settings.validate({"power_guard": False}, bot))
                    harness = AppHarness(self, (wheel, "SET", wheel, "Q"))
                    answers = iter(("1", "1", wheel, "0", "0"))
                    observed = []
                    output = harness.gpio.output

                    def record(pin, value):
                        output(pin, value)
                        if pin == motor.STBY and value == 1 and harness.drivers[-1].pattern is not None:
                            observed.append(tuple((harness.gpio.values[a], harness.gpio.values[b])
                                                  for a, b, _ in motor.MOTOR_PINS))

                    def drive(command, sequence, motors, *args):
                        SingleMotorTests(motors, FakeKeys(harness.clock), harness.clock,
                                         lambda message: None).run_test(command)

                    with harness.environment(), patch.object(motor, "prompt_line", side_effect=lambda _: next(answers)), \
                            patch.object(app, "dispatch", side_effect=drive), \
                            patch.object(harness.gpio, "output", side_effect=record):
                        self.assertEqual(app.main(["--bot", bot, "--settings", str(path)]), 0)
                    self.assertEqual(len(harness.drivers), 2)
                    self.assertEqual(len(observed), 2)
                    self.assertEqual(observed[0][index], observed[1][index][::-1])
                    for other in range(4):
                        if other != index:
                            self.assertEqual(observed[0][other], observed[1][other])
                    saved = settings.load_settings(path, bot)
                    self.assertEqual(saved["reversed"], tuple(i == index for i in range(4)))
                    self.assertEqual(saved["robot_id"], bot + "rpm")
                    self.assertTrue(all(not driver.ready for driver in harness.drivers))
                    self.assertIn("applying them to this robot now", harness.stdout.getvalue())

    def test_wheel_shortcut_preserves_other_flags_and_can_toggle_back(self):
        path = self.root / "custom.json"
        config = settings.validate({"reversed": (True, False, True, False)})
        settings.save_settings(path, config)
        answers = iter(("1", "1", "FR", "1", "BL", "0", "0"))
        result = terminal_ui.edit_settings(config, path, read=lambda _: next(answers), write=lambda _: None)
        self.assertEqual(result["reversed"], (True, True, False, False))
        self.assertEqual(config["reversed"], (True, False, True, False))

    def test_failed_cleanup_after_set_prevents_reloading_saved_wiring(self):
        harness = SwitchingHarness(self, self.root, commands=("SET", "Q"))
        harness.close_failures["battery"] = False
        harness.edit = lambda config, path: dict(config, reversed=(True, False, False, False))
        with harness.environment():
            self.assertEqual(app.main(["--battery-profile", "none"]), 1)
        self.assertEqual(len(harness.drivers), 1)
        self.assertEqual(harness.selection_count, 1)
        self.assertEqual(settings.load_settings(self.root / "profiles" / "500rpm.json")["reversed"][0], True)

    def test_tagged_recent_profiles_migrate_both_bots_without_resetting_reversals(self):
        previous = self.root / "MPU6050_Motor_Test_v9_2" / "profiles"
        previous.mkdir(parents=True)
        current = self.root / "MPU6050_Motor_Test_v9_3"
        older = self.root / "MPU6050_Motor_Test_v9_1" / "profiles"
        older.mkdir(parents=True)
        for bot in ("500", "100"):
            source = previous / f"{bot}rpm.json"
            config = settings.validate({"reversed": (False, True, True, False), "power_guard": False}, bot)
            settings.save_settings(source, config)
            settings.save_settings(older / f"{bot}rpm.json", settings.validate({}, bot))
            original = source.read_bytes()
            preview = robot_profiles.load_profile(current, bot, persist=False)[1]
            self.assertEqual(preview, config)
            if bot == "500":
                self.assertFalse(current.exists())
            destination, loaded, _ = robot_profiles.load_profile(current, bot)
            self.assertEqual(loaded, config)
            self.assertEqual(source.read_bytes(), original)
            self.assertEqual(settings.load_settings(destination, bot), config)


class INAReliabilityTests(unittest.TestCase):
    def test_readback_error_reports_expected_actual_and_address(self):
        bus = FakeBatteryBus(FakeClock())
        bus.write_i2c_block_data = lambda *args: None
        with self.assertRaisesRegex(battery.BatteryFault, "wrote 0x2E60, read 0x399F at 0x41"):
            battery.INA219(bus, address=0x41).initialize()

    def test_worker_retries_transient_initialization_errors_only(self):
        sampled = threading.Event()
        bus = Mock()
        sensor = Mock()
        sensor.initialize.side_effect = [OSError("transient bus error"), None]

        def read():
            sampled.set()
            return reading(time.monotonic())

        sensor.read_sample.side_effect = read
        with patch.object(battery, "INA219", return_value=sensor):
            stream = battery.BatteryStream(bus_factory=lambda _: bus)
            try:
                self.assertTrue(sampled.wait(1))
                deadline = time.monotonic() + 1
                while stream.latest() is None and time.monotonic() < deadline:
                    time.sleep(0.001)
                self.assertIsNotNone(stream.latest())
                self.assertEqual(sensor.initialize.call_count, 2)
            finally:
                self.assertTrue(stream.close())
        bus.close.assert_called_once()

    def test_worker_reports_bus_address_and_does_not_hide_midrun_failure(self):
        bus = Mock()
        sensor = Mock()
        sensor.read_sample.side_effect = OSError(121, "Remote I/O error")
        with patch.object(battery, "INA219", return_value=sensor):
            stream = battery.BatteryStream(bus_number=3, address=0x41, bus_factory=lambda _: bus)
            stream._thread.join(timeout=1)
            try:
                with self.assertRaisesRegex(battery.BatteryFault, "I2C bus 3, address 0x41.*Remote I/O error"):
                    stream.latest()
                self.assertEqual(sensor.read_sample.call_count, 1)
            finally:
                self.assertTrue(stream.close())

    def test_bat_and_guard_cannot_create_a_second_owner_after_blocked_read(self):
        factory = Mock()
        session = battery.BatterySession(stream_factory=factory)
        stuck = Mock()
        stuck.close.return_value = False
        session.stream = stuck
        with self.assertRaisesRegex(battery.BatteryFault, "still blocked"):
            power_guard.PowerGuard.from_settings(settings.DEFAULTS,
                stream_factory=lambda: session.start(period=power_guard.GUARD_PERIOD))
        factory.assert_not_called()
        self.assertIs(session.stream, stuck)
        self.assertFalse(session.close())

    def test_guard_stream_is_tracked_by_battery_owner_through_close_failure(self):
        stuck = Mock()
        stuck.close.return_value = False
        session = battery.BatterySession(stream_factory=Mock(return_value=stuck))
        guard = power_guard.PowerGuard.from_settings(settings.DEFAULTS,
            stream_factory=lambda: session.start(period=power_guard.GUARD_PERIOD))
        self.assertIs(session.stream, guard.stream)
        self.assertFalse(guard.close())
        self.assertFalse(session.close())
        self.assertIs(session.stream, stuck)

    def test_wait_ready_checks_freshness_and_is_cancellable(self):
        clock = FakeClock()
        stream = ScriptedStream(clock)
        guard = make_guard(clock, stream)
        for age in (0.6, -1.0):
            stream.age = age
            with self.assertRaises(power_guard.PowerFault):
                guard.wait_ready()
        stream.none = True
        def cancel():
            raise motor.StopRequested()
        with self.assertRaises(motor.StopRequested):
            guard.wait_ready(poll=cancel)
        self.assertEqual(clock.now, 0)

    def test_sensor_error_before_first_pwm_is_not_ignored(self):
        clock = FakeClock()
        stream = ScriptedStream(clock)
        guard = make_guard(clock, stream)
        guard.wait_ready()
        stream.error = "lost sensor after startup"
        motors = FakeMotors()
        with self.assertRaisesRegex(power_guard.PowerFault, "lost sensor"):
            guard.check(motors)
        self.assertEqual(motors.duties, [0] * 4)
        self.assertEqual(motors.stops, [False])

    def test_reversed_shunt_cannot_bypass_overcurrent(self):
        clock = FakeClock()
        stream = ScriptedStream(clock)
        stream.current = -6
        guard = make_guard(clock, stream)
        motors = FakeMotors()
        motors.duties = [25] * 4
        guard.check(motors)
        clock.sleep(0.21)
        with self.assertRaisesRegex(power_guard.PowerFault, "Overcurrent: 6.00 A"):
            guard.check(motors)
        self.assertIn("peak 6.00 A", guard.summary())

    def test_saturation_limit_message_uses_configured_shunt(self):
        stream = ScriptedStream(FakeClock())
        stream.error = "INA219 shunt reading is at its +/-80 mV limit"
        guard = power_guard.PowerGuard.from_settings(settings.DEFAULTS,
            shunt_ohms=0.1, stream_factory=lambda: stream)
        with self.assertRaisesRegex(power_guard.PowerFault, "about 0.8 A"):
            guard.check(FakeMotors())

    def test_standalone_check_never_opens_gpio_or_creates_profiles(self):
        for error in (None, battery.BatteryFault("INA219 on I2C bus 1, address 0x40: No device")):
            with self.subTest(error=error), tempfile.TemporaryDirectory() as folder:
                stream = Mock()
                stream.latest.side_effect = error
                stream.latest.return_value = reading(time.monotonic())
                stream.close.return_value = True
                session = battery.BatterySession(stream_factory=lambda *args: stream)
                with patch.object(app, "BatterySession", return_value=session), \
                        patch.object(app, "PACKAGE_DIR", Path(folder)), \
                        patch.object(motor, "MotorDriver") as driver, \
                        contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(app.main(["--check-ina"]), 0 if error is None else 1)
                driver.assert_not_called()
                self.assertEqual(list(Path(folder).iterdir()), [])
                self.assertIsNone(session.stream)


class AppINARecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "settings.json"
        self.addCleanup(settings.apply_settings, deepcopy(settings.DEFAULTS))

    def test_ina_failure_leaves_diagnostics_and_set_accessible_and_explicit_off_applies(self):
        harness = AppHarness(self, ("FL", "IRC", "SET", "FL", "Q"))
        created = []
        output_after_failure = []

        def factory(config, **kwargs):
            stream = ScriptedStream(time)
            stream.error = "I2C bus 1 address 0x40: Remote I/O error"
            guard = power_guard.PowerGuard.from_settings(config, stream_factory=lambda: stream)
            created.append(guard)
            return guard

        def before(number, status):
            if number == 2:
                output_after_failure.append(list(harness.gpio.pwm_writes))
                self.assertEqual(harness.gpio.values[motor.STBY], 0)
                self.assertIn("Remote I/O error", status["last_result"])

        def edit(config, path, read):
            result = dict(config, power_guard=False)
            settings.save_settings(path, result)
            return result

        harness.before_choice = before
        with harness.environment(), patch.object(app, "PowerGuard", types.SimpleNamespace(from_settings=factory)), \
                patch.object(app, "edit_settings", side_effect=edit):
            self.assertEqual(app.main(["--bot", "500", "--settings", str(self.path)]), 0)
        self.assertEqual(len(created), 1)
        self.assertTrue(created[0].stream.closed)
        self.assertTrue(all(duty == 0 for _, _, duty in output_after_failure[0]))
        self.assertIn("IRC", harness.events)
        self.assertEqual(len(harness.drivers), 2)
        self.assertTrue(any(duty > 0 for _, _, duty in harness.gpio.pwm_writes))
        self.assertFalse(settings.load_settings(self.path)["power_guard"])
        self.assertIn("Power guard OFF", harness.stdout.getvalue())

    def test_blocked_guard_cannot_be_bypassed_by_bot_switch(self):
        from fake_power import FakeGuard
        class StuckGuard(FakeGuard):
            def close(self):
                return False

        harness = AppHarness(self, ("FC", "BOT", "Q"))
        with harness.environment(), patch.object(app, "PowerGuard", StuckGuard), \
                patch.object(app, "choose_robot") as chooser:
            self.assertEqual(app.main(["--bot", "500", "--settings", str(self.path)]), 1)
        chooser.assert_not_called()
        self.assertEqual(len(harness.drivers), 1)
        self.assertIn("restart the program", harness.stdout.getvalue())


if __name__ == "__main__":
    unittest.main(verbosity=2)
