"""Offline checks for the new 2.09-prime connecting code."""
import argparse
import contextlib
import hashlib
import importlib
import io
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import Mock, patch
from test_prime_gpio import FakeGPIO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
app = importlib.import_module('main')


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.output = io.StringIO()
        self.capture = contextlib.redirect_stdout(self.output)
        self.capture.__enter__()
        self.addCleanup(self.capture.__exit__, None, None, None)

    def test_all_menu_commands_reach_existing_dispatch_set(self):
        exposed = {code for _, commands in app.CATEGORIES.values() for code, _ in commands}
        self.assertEqual(exposed, app.COMMANDS)
        for category, (_, commands) in app.CATEGORIES.items():
            self.assertEqual(app.resolve_choice(category, None), (category, None))
            for index, (code, _) in enumerate(commands, 1):
                self.assertEqual(app.resolve_choice(str(index), category), (category, code))
                self.assertEqual(app.resolve_choice(code.lower(), category), (category, code))

    def test_menu_home_help_quit_and_invalid_numbers(self):
        self.assertEqual(app.resolve_choice('0', '2'), (None, None))
        self.assertEqual(app.resolve_choice('100', '2'), ('2', ''))
        for command in ['H', 'Q']:
            self.assertEqual(app.resolve_choice(command, '4'), ('4', command))

    def test_preview_does_not_open_gpio_camera_or_i2c(self):
        with patch.object(app.motor, 'MotorDriver', side_effect=AssertionError('hardware opened')):
            self.assertEqual(app.main(['--preview']), 0)
        text = self.output.getvalue()
        self.assertIn('500 RPM bot', text)
        self.assertIn('100 RPM bot', text)
        self.assertNotIn('picamera2', sys.modules)
        self.assertNotIn('adafruit_vl53l0x', sys.modules)
        self.assertNotIn('RPi.GPIO', sys.modules)

    def test_each_preset_applies_motor_and_imu_values(self):
        for bot_id, preset in app.motion_profile.PRESETS.items():
            app.apply_preset(bot_id)
            self.assertEqual(app.motor.LEFT_SPEED, preset['left_speed'])
            self.assertEqual(app.motor.RIGHT_SPEED, preset['right_speed'])
            self.assertEqual(app.single_motor.TEST_DUTY, preset['single_motor_duty'])
            self.assertEqual(app.imu_tests.IMU_DRIVE_DUTY, preset['imu_drive_duty'])
            self.assertEqual(app.imu_tests.TURN_MAX_DUTY, preset['imu_turn_max_duty'])
        app.apply_preset('500rpm')

    def test_motor_imu_and_wheel_commands_use_existing_classes(self):
        motors, keys, session = Mock(), Mock(), Mock()
        with patch.object(app, 'SingleMotorTests') as wheels, patch.object(app, 'IMUTests') as imu:
            app.dispatch('FL', motors, keys, session)
            wheels.return_value.run_test.assert_called_once_with('FL')
            app.dispatch('IF', motors, keys, session)
            imu.return_value.run_test.assert_called_once_with('IF')
        with patch.object(app.motor, 'Controller') as controller, patch.object(app, 'TurnYawLogger') as logger:
            app.dispatch('R', motors, keys, session)
            controller.return_value.run_mode.assert_called_once_with('R')
            logger.return_value.start.assert_called_once_with()

    def test_sensor_commands_route_to_shared_readout(self):
        with patch.object(app, 'sensor_readout') as readout:
            for code in ('TOF', 'VISION'):
                app.dispatch(code, 'motors', 'keys', 'session')
                readout.assert_called_with(code, 'motors', 'keys')

    def test_tof_one_read_then_stop_and_release(self):
        module, keys, motors = Mock(), Mock(), Mock()
        keys.poll.side_effect = ['', '', '', 's']
        module.read_tof.return_value = {'obstacle': True}
        events = []
        motors.stop.side_effect = lambda **kw: events.append('stop')
        module.setup_tof.side_effect = lambda: events.append('setup')
        with patch.dict(sys.modules, {'tof_sensor': module, 'vision': module}), patch.object(app.time, 'sleep'):
            with self.assertRaises(app.motor.StopRequested):
                app.sensor_readout('TOF', motors, keys)
        self.assertEqual(events[:2], ['stop', 'setup'])
        module.print_tof_data.assert_called_once_with({'obstacle': True})
        module.reset_tof_state.assert_called_once_with()
        module.stop_tof.assert_called_once_with()
        motors.set_duties.assert_not_called()
        motors.stop.assert_any_call(brake=False)

    def test_camera_stop_closes_and_clears_owner(self):
        module, camera, keys = Mock(), Mock(), Mock()
        module.picam2 = camera
        keys.poll.side_effect = ['', '', '', 's']
        with patch.dict(sys.modules, {'tof_sensor': module, 'vision': module}):
            with self.assertRaises(app.motor.StopRequested):
                app.sensor_readout('VISION', Mock(), keys)
        module.read_vision.assert_called_once_with()
        camera.stop.assert_called_once_with()
        camera.close.assert_called_once_with()
        self.assertIsNone(module.picam2)

    def test_missing_optional_dependency_returns_without_motion(self):
        motors = Mock()
        with patch.dict(sys.modules, {'vision': None}):
            self.assertFalse(app.sensor_readout('VISION', motors, Mock()))
        motors.stop.assert_called_once_with(brake=False)
        self.assertIn('ModuleNotFoundError', self.output.getvalue())

    def test_tof_setup_failure_releases_partial_sensor(self):
        module, keys = Mock(), Mock()
        keys.poll.return_value = ''
        module.setup_tof.side_effect = OSError('bus')
        with patch.dict(sys.modules, {'tof_sensor': module, 'vision': module}):
            self.assertFalse(app.sensor_readout('TOF', Mock(), keys))
        module.stop_tof.assert_called_once_with()

    def test_camera_setup_failure_closes_partial_camera(self):
        module, camera, keys = Mock(), Mock(), Mock()
        module.picam2 = camera
        keys.poll.return_value = ''
        module.setup_camera.side_effect = OSError('configuration')
        with patch.dict(sys.modules, {'tof_sensor': module, 'vision': module}):
            self.assertFalse(app.sensor_readout('VISION', Mock(), keys))
        camera.close.assert_called_once_with()
        self.assertIsNone(module.picam2)

    def test_read_failure_cleans_up_both_sensor_types(self):
        for command in ('TOF', 'VISION'):
            module, keys, camera = Mock(), Mock(), Mock()
            module.picam2 = camera
            keys.poll.return_value = ''
            module.read_tof.side_effect = TimeoutError('read')
            module.read_vision.side_effect = TimeoutError('read')
            with patch.dict(sys.modules, {'tof_sensor': module, 'vision': module}):
                self.assertFalse(app.sensor_readout(command, Mock(), keys))
            if command == 'TOF':
                module.stop_tof.assert_called_once_with()
            else:
                camera.close.assert_called_once_with()

    def test_interrupt_also_releases_sensor(self):
        module, keys = Mock(), Mock()
        keys.poll.return_value = ''
        module.read_tof.side_effect = KeyboardInterrupt()
        with patch.dict(sys.modules, {'tof_sensor': module, 'vision': module}):
            with self.assertRaises(KeyboardInterrupt):
                app.sensor_readout('TOF', Mock(), keys)
        module.stop_tof.assert_called_once_with()

    def test_cleanup_failure_is_fatal_even_when_value_error(self):
        module, keys = Mock(), Mock()
        keys.poll.side_effect = ['', 's']
        module.stop_tof.side_effect = ValueError('cleanup')
        with patch.dict(sys.modules, {'tof_sensor': module, 'vision': module}):
            with self.assertRaisesRegex(RuntimeError, 'cleanup failed'):
                app.sensor_readout('TOF', Mock(), keys)

    def test_camera_close_attempted_even_if_stop_fails(self):
        module, camera, keys = Mock(), Mock(), Mock()
        module.picam2 = camera
        camera.stop.side_effect = OSError('stop')
        keys.poll.side_effect = ['', 's']
        with patch.dict(sys.modules, {'tof_sensor': module, 'vision': module}):
            with self.assertRaises(RuntimeError):
                app.sensor_readout('VISION', Mock(), keys)
        camera.close.assert_called_once_with()

    def run_menu(self, choices, motors=None, session=None, dispatch=None):
        motors = motors or Mock()
        session = session or Mock(calibration=None)
        args = argparse.Namespace(bus=1, address=0x68, bot_id='500rpm')
        with patch.object(app.motor, 'MotorDriver', return_value=motors), \
             patch.object(app, 'Session', return_value=session), \
             patch.object(app.motor, 'prompt_line', side_effect=choices), \
             patch.object(app.motor, 'Keyboard', return_value=contextlib.nullcontext(Mock())), \
             patch.object(app, 'dispatch', dispatch or Mock()):
            return app.run(args, Mock())

    def test_full_menu_routes_numbered_and_direct_commands(self):
        dispatch = Mock()
        self.assertEqual(self.run_menu(['3', '1', '0', '4', '1', '0', '2', '1', 'FL', 'H', 'Q'], dispatch=dispatch), 0)
        self.assertEqual([call.args[0] for call in dispatch.call_args_list], ['TOF', 'VISION', 'ID', 'FL'])

    def test_cleanup_failure_ends_menu_before_next_command(self):
        dispatch = Mock(side_effect=RuntimeError('cleanup failed'))
        with contextlib.redirect_stderr(self.output):
            self.assertEqual(self.run_menu(['TOF', 'F', 'Q'], dispatch=dispatch), 1)
        dispatch.assert_called_once()

    def test_shutdown_motor_before_imu_and_both_attempted(self):
        for failing in ('motor', 'imu', None):
            events = []
            motors, session = Mock(), Mock(calibration=None)
            def close_motor():
                events.append('motor')
                if failing == 'motor':
                    raise OSError('gpio cleanup')
                return True
            def close_imu():
                events.append('imu')
                if failing == 'imu':
                    raise OSError('imu cleanup')
                return True
            motors.close.side_effect = close_motor
            session.close.side_effect = close_imu
            with contextlib.redirect_stderr(self.output):
                self.assertEqual(self.run_menu(['Q'], motors, session), 0 if failing is None else 1)
            self.assertEqual(events, ['motor', 'imu'])

    def test_signal_wrapper_restores_previous_handlers(self):
        names = [name for name in app.STOP_SIGNALS if hasattr(app.signal, name)]
        with patch.object(app.signal, 'signal', return_value='old') as register:
            with app.stop_on_signals():
                pass
        self.assertEqual(register.call_count, len(names) * 2)
        for name in names:
            register.assert_any_call(getattr(app.signal, name), 'old')

    def test_source_manifest_matches_git_and_unchanged_files(self):
        manifest = json.loads((ROOT/'SOURCES.json').read_text())
        git = ['git', '-C', str(ROOT)]
        for name, record in manifest['files'].items():
            original = subprocess.check_output(git+['show', record['commit']+':'+name])
            self.assertEqual(hashlib.sha256(original).hexdigest(), record['source_sha256'], name)
            if record['state'] == 'unchanged':
                self.assertEqual((ROOT/name).read_bytes(), original, name)
        subprocess.run(git+['merge-base', '--is-ancestor', manifest['base_commit'], 'HEAD'], check=True)

    def test_every_python_file_compiles_without_hardware_imports(self):
        for path in ROOT.glob('*.py'):
            compile(path.read_bytes(), str(path), 'exec')

    def test_real_driver_quit_releases_gpio_through_launcher(self):
        gpio = FakeGPIO()
        args = argparse.Namespace(bus=1, address=0x68, bot_id='500rpm')
        with patch.object(app, 'Session', return_value=Mock(calibration=None)), \
             patch.object(app.motor, 'prompt_line', return_value='Q'):
            self.assertEqual(app.run(args, gpio), 0)
        self.assertEqual(len([e for e in gpio.events if e[0] == 'cleanup']), 13)
        self.assertTrue(all(value == gpio.IN for value in gpio.functions.values()))

    def test_real_gpio_fault_exits_launcher_and_cleans_all_pins(self):
        gpio = FakeGPIO()
        args = argparse.Namespace(bus=1, address=0x68, bot_id='500rpm')
        gpio.failures[('duty', app.motor.MOTOR_PINS[1][2], 20.0)] = 1
        commands = []
        def drive(command, motors, keys, session):
            commands.append(command)
            motors.set_pattern(app.motor.FORWARD)
            motors.set_duties((20,) * 4)
        with patch.object(app, 'Session', return_value=Mock(calibration=None)), \
             patch.object(app.motor, 'prompt_line', side_effect=['F', 'F', 'Q']), \
             patch.object(app.motor, 'Keyboard', return_value=contextlib.nullcontext(Mock())), \
             patch.object(app, 'dispatch', side_effect=drive), \
             contextlib.redirect_stderr(self.output):
            self.assertEqual(app.run(args, gpio), 1)
        self.assertEqual(commands, ['F'])
        self.assertEqual(len([e for e in gpio.events if e[0] == 'cleanup']), 13)
        self.assertTrue(all(value == gpio.IN for value in gpio.functions.values()))

    def test_interrupt_during_real_driver_motion_runs_shutdown(self):
        gpio = FakeGPIO()
        args = argparse.Namespace(bus=1, address=0x68, bot_id='500rpm')
        def drive(command, motors, keys, session):
            motors.set_pattern(app.motor.FORWARD)
            motors.set_duties((20,) * 4)
            raise KeyboardInterrupt()
        with patch.object(app, 'Session', return_value=Mock(calibration=None)), \
             patch.object(app.motor, 'prompt_line', return_value='F'), \
             patch.object(app.motor, 'Keyboard', return_value=contextlib.nullcontext(Mock())), \
             patch.object(app, 'dispatch', side_effect=drive):
            self.assertEqual(app.run(args, gpio), 0)
        self.assertTrue(all(pwm.stopped for pwm in gpio.pwms))
        self.assertTrue(all(value == gpio.IN for value in gpio.functions.values()))


if __name__ == '__main__':
    unittest.main(verbosity=2)
