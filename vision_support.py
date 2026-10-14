"""Optional headless Ryan vision. Camera/detection never runs in the motor loop.

With VT on, the camera decides each front stop line in the lane modes (TrafficLight):
a confirmed green lets the robot cross it, a red or yellow stops it there until green.
"""

from collections import deque
from dataclasses import dataclass
import math
import multiprocessing
import os
from queue import Empty, Full
import time

import vision_settings as settings

VISION_MAX_AGE = 0.75
VISION_START_TIMEOUT = 30.0
VISION_LABELS = {"stop_sign", "red_light", "yellow_light", "green_light"}
# Lane modes that start the camera with VT on: the light decides each front line.
LIGHT_COMMANDS = {"F", "FA", "C", "FG", "CG", "FC"}
# The camera worker runs at a lower CPU priority (Linux nice, 0-19) so the motor loop in
# the main process gets the CPU first; detection takes a little longer under load.
WORKER_NICE = 10
# A confirmed green still lets the robot cross this long after its last confirming frame,
# so a lamp that drops out of a frame or two right at the line does not stop the robot.
GREEN_HOLD = 1.0
# Stopped at a front line with no course stop to wait out, the camera looks this long
# (about six frames at VISION_PROCESS_HZ) before the robot goes on, waits or ends.
LOOK_SECONDS = 0.6
RVD_MAX_AGE = 5.0  # RVD (home testing): the stale limit, since saving a snapshot stalls the camera
STOP_LIGHTS = {"red_light", "yellow_light"}


class VisionFault(Exception):
    pass


@dataclass(frozen=True)
class CameraOptions:
    backend: str = settings.CAMERA_BACKEND
    index: int = settings.CAMERA_INDEX
    rotate_180: bool = settings.CAMERA_ROTATE_180
    auto_exposure: bool = False
    exposure_us: int = settings.CAMERA_EXPOSURE_US
    analogue_gain: float = settings.CAMERA_ANALOGUE_GAIN


@dataclass(frozen=True)
class VisionSample:
    sequence: int
    timestamp: float  # Before capture; includes acquisition and processing time.
    detections: tuple  # (label, bounding-box area fraction)
    processing_seconds: float
    locations: tuple = ()  # Optional (label, LEFT/CENTER/RIGHT, (x, y)) diagnostics.
    debug: tuple = ()  # RVD only: rv_debug's lines for this frame.


def check_vision_sample(sample, now, max_age=None):
    if sample is None:
        raise VisionFault("No camera detection result is available")
    if (not isinstance(sample, VisionSample) or sample.sequence < 1
            or not math.isfinite(sample.timestamp) or not math.isfinite(now)
            or not math.isfinite(sample.processing_seconds) or sample.processing_seconds < 0):
        raise VisionFault("Invalid camera result")
    if not -0.01 <= now - sample.timestamp <= (VISION_MAX_AGE if max_age is None else max_age):
        raise VisionFault("Camera result is stale; the camera run stopped")
    for label, area in sample.detections:
        if label not in VISION_LABELS or not math.isfinite(area) or not 0 <= area <= 1:
            raise VisionFault("Invalid detector output")
    for label, position, center in sample.locations:
        if (label not in {item[0] for item in sample.detections}
                or position not in {"LEFT", "CENTER", "RIGHT"}
                or len(center) != 2 or any(not math.isfinite(v) or v < 0 for v in center)):
            raise VisionFault("Invalid detector location")
    return sample


class HeadlessCamera:
    """Used only in the child process. No preview window or GUI calls."""

    def __init__(self, options):
        self.options = options
        self.picam = self.capture = None
        import cv2
        self.cv2 = cv2
        try:
            if options.backend == "csi":
                from picamera2 import Picamera2
                self.picam = Picamera2(options.index)
                controls = {"FrameRate": float(settings.CAMERA_FPS)}
                if not options.auto_exposure:
                    controls.update(AeEnable=False, ExposureTime=options.exposure_us,
                                    AnalogueGain=options.analogue_gain)
                config = self.picam.create_video_configuration(
                    main={"size": (settings.CAMERA_WIDTH, settings.CAMERA_HEIGHT), "format": "RGB888"},
                    raw={"size": settings.CAMERA_SENSOR_SIZE}, controls=controls,
                    buffer_count=3, queue=False)
                self.picam.configure(config)
                self.picam.start(show_preview=False)
            elif options.backend == "usb":
                self.capture = cv2.VideoCapture(options.index, cv2.CAP_V4L2)
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, settings.CAMERA_WIDTH)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, settings.CAMERA_HEIGHT)
                self.capture.set(cv2.CAP_PROP_FPS, settings.CAMERA_FPS)
                self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not self.capture.isOpened():
                    raise VisionFault("USB camera did not open")
            else:
                raise VisionFault("Camera backend must be csi or usb")
        except BaseException:
            self.close()
            raise

    def read(self):
        if self.picam is not None:
            # Picamera2 RGB888 arrays are B,G,R, matching OpenCV.
            frame = self.picam.capture_array("main")
        else:
            ok, frame = self.capture.read()
            if not ok:
                raise VisionFault("USB camera frame read failed")
        if frame is None or frame.size == 0 or len(frame.shape) != 3 or frame.shape[2] != 3:
            raise VisionFault("Camera returned an empty or unsupported frame")
        return self.cv2.flip(frame, -1) if self.options.rotate_180 else frame

    def close(self):
        if self.picam is not None:
            try:
                self.picam.stop()
            finally:
                self.picam.close()
                self.picam = None
        if self.capture is not None:
            self.capture.release()
            self.capture = None


def _publish(queue, value):
    # The parent runs faster than vision; keep only a short bounded queue.
    try:
        queue.put_nowait(value)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        try:
            queue.put_nowait(value)
        except Full:
            pass


def vision_worker(options, queue, done, snap=None, debug=False):
    """Camera process. RVD (home testing) adds ``debug``: rv_debug's lines on every frame, and a
    ``snap`` event that saves the next frame plus an annotated copy to ~/rv_snapshots."""
    camera = None
    try:
        try:
            os.nice(WORKER_NICE)
        except (AttributeError, OSError):
            pass  # not Linux, or not permitted: run at normal priority
        import cv2
        from ryan_vision.detector_geometric import GeometricDetector
        cv2.setNumThreads(2)
        detector = GeometricDetector(settings)
        camera = HeadlessCamera(options)
        # Capture through startup while auto white balance settles. No frame
        # is marked fresh until it is actually acquired and processed.
        warm_until = time.monotonic() + 1.0
        sequence = 0
        while not done.is_set():
            started = time.monotonic()
            frame = camera.read()
            if started < warm_until:
                continue
            results = detector.detect(frame)
            detections = tuple((d.label, float(d.area_frac)) for d in results)
            locations = tuple((d.label, d.position, tuple(int(v) for v in d.center)) for d in results)
            notes = blobs = ()
            if debug:
                import rv_debug
                blobs = rv_debug.light_debug(frame)
                notes = rv_debug.describe(frame, blobs)
            sequence += 1
            elapsed = time.monotonic() - started
            _publish(queue, VisionSample(sequence, started, detections, elapsed, locations, notes))
            if snap is not None and snap.is_set():
                # After publishing: writing two JPEGs to the SD card takes a while on the Pi.
                snap.clear()
                _publish(queue, ("snap", str(rv_debug.save_snapshot(frame, blobs))))
            done.wait(max(0, 1.0 / settings.VISION_PROCESS_HZ - elapsed))
    except Exception as exc:
        _publish(queue, ("error", f"{type(exc).__name__}: {exc}"))
    finally:
        if camera is not None:
            camera.close()


class VisionStream:
    """Spawn a camera-only process; a stalled camera cannot own motor GPIO."""

    def __init__(self, options=CameraOptions(), context=None, worker=vision_worker, debug=False):
        context = context or multiprocessing.get_context("spawn")
        self.queue = context.Queue(maxsize=4)
        self.done = context.Event()
        self.snap = context.Event() if debug else None  # RVD: set to save a snapshot
        self.snapshots = []  # RVD: annotated snapshot paths the worker reported
        args = (options, self.queue, self.done) + ((self.snap, True) if debug else ())
        self.process = context.Process(target=worker, args=args, daemon=True)
        self.sample = self.error = None
        self.closed = False
        self.process.start()

    def latest(self):
        if self.closed:
            raise VisionFault("Camera worker is closed")
        for _ in range(4):
            try:
                value = self.queue.get_nowait()
            except Empty:
                break
            if isinstance(value, tuple) and len(value) == 2 and value[0] == "error":
                self.error = value[1]
            elif isinstance(value, tuple) and len(value) == 2 and value[0] == "snap":
                self.snapshots.append(value[1])
            elif isinstance(value, VisionSample):
                self.sample = value
            else:
                self.error = "Unexpected camera-worker message"
        if self.error is not None:
            raise VisionFault(self.error)
        if not self.process.is_alive():
            raise VisionFault("Camera worker exited; check camera and OpenCV/Picamera2 dependencies")
        return self.sample

    def close(self):
        if self.closed:
            return True
        self.done.set()
        self.process.join(timeout=0.25)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=0.25)
        if self.process.is_alive() and hasattr(self.process, "kill"):
            self.process.kill()
            self.process.join(timeout=0.25)
        if self.process.is_alive():
            return False  # Never start a duplicate owner of an unresponsive camera.
        self.queue.cancel_join_thread()
        self.queue.close()
        self.closed = True
        return True


def act_size(label):
    """Bounding-box fraction a detection must reach to count (SET -> Camera), not a distance."""
    return settings.STOPSIGN_ACT_AREA_FRAC if label == "stop_sign" else settings.LIGHT_ACT_AREA_FRAC


class TrafficLight:
    """VT: the camera's say over the front stop line in every lane mode.

    The front IR stop line stays ARMED by default, exactly as with the camera off. A green
    light confirmed in 2 of the last 3 fresh frames (at its action size) overrides it: the
    robot notes the line and drives across. A red or yellow confirmed after that re-arms it,
    so the robot stops at the line (PRD: enter an intersection only on green). In any one
    window red outranks yellow, and both outrank green; a lamp flickering between red and
    yellow counts as red, as in v9.5's FC gate. A green lets the robot cross for GREEN_HOLD
    after its last confirming frame, unless the newest frame shows red or yellow.

    Stopped at a line, only what the camera confirms while stopped counts: a red or yellow
    waits there for a fresh green, a green goes on, nothing seen is the mode's usual stop.
    Once the robot goes on across a line, the light it just passed is ignored until no light
    is confirmed (it has left the frame or shrunk below the action size), so its green can
    never carry the robot over the next intersection's line. That can cost a stop and a
    look; it never skips one. A stop sign confirmed on the approach marks the next stop line
    as the end of the run (PRD: the course ends stopped at the stop sign).
    """

    look_seconds = LOOK_SECONDS

    def __init__(self, stream, log=print, max_age=None):
        self.stream, self.log = stream, log
        self.max_age = max_age  # RVD only: a longer stale limit while a snapshot is written
        self.previous = None
        self.history = deque(maxlen=3)
        self.confirmed = set()
        self.state = None          # "red", "yellow" or "green": the newest confirmed light
        self.state_at = None       # when it was last confirmed
        self.looking_since = None  # set while stopped at a line: only later confirmations decide
        self.passed = False        # from going on across a line until that light is out of view
        self.stop_sign = False
        self.said = None

    def update(self, now):
        """Read the newest camera result; called every control tick. Raises VisionFault if stale."""
        sample = check_vision_sample(self.stream.latest(), now, self.max_age)
        if self.previous is not None:
            if sample.sequence < self.previous.sequence or sample.timestamp < self.previous.timestamp:
                raise VisionFault("Camera results arrived out of order")
            if sample.sequence == self.previous.sequence:
                return
            if sample.timestamp <= self.previous.timestamp:
                raise VisionFault("Camera timestamp did not advance")
        self.previous = sample
        self.history.append({label for label, area in sample.detections if area >= act_size(label)})
        self.confirmed = {label for label in VISION_LABELS
                          if sum(label in frame for frame in self.history) >= 2}
        light = self.light()
        if self.passed:
            if light is None:
                self.passed = False  # the light just passed is out of view
        elif light is not None:
            self.state, self.state_at = light, now
            if light != self.said:
                self.said = light
                self.log(f"Camera: {light} light, front stop line " + ("overridden" if light == "green"
                                                                       else "armed"))
        if "stop_sign" in self.confirmed and not self.stop_sign:
            self.stop_sign = True
            self.log("Camera: stop sign, the next front line ends the run")

    def light(self):
        """The light the last 3 frames confirm: "red", "yellow", "green" or None."""
        if "red_light" in self.confirmed:
            return "red"
        if "yellow_light" in self.confirmed:
            return "yellow"
        if sum(bool(frame & STOP_LIGHTS) for frame in self.history) >= 2:
            return "red"  # one lamp read as red, then yellow: stop either way
        if "green_light" in self.confirmed:
            return "green"
        return None

    def newest_stop_light(self):
        """A newest qualifying red/yellow vetoes green before two-frame confirmation."""
        newest = self.history[-1] if self.history else set()
        if "red_light" in newest:
            return "red"
        if "yellow_light" in newest:
            return "yellow"
        return None

    def unlit_crossing(self):
        """CG intermediate lines only: fresh frames contain no actionable lamp.

        poll() validates freshness before this decision. Keep a previously seen
        red/yellow latched until green; losing a lamp must not release that stop.
        """
        lamps = {"red_light", "yellow_light", "green_light"}
        return (self.previous is not None and bool(self.history)
                and not self.stop_sign and self.state not in ("red", "yellow")
                and not any(frame & lamps for frame in self.history))

    def crossing(self, now):
        """At a front line while driving: True when a recent green lets the robot cross it."""
        return (self.state == "green" and not self.stop_sign
                and now - self.state_at <= GREEN_HOLD + 1e-9
                and self.newest_stop_light() is None)

    def at_line(self, now):
        """The robot has stopped at a front line: what it sees from here is this line's light."""
        self.passed = False
        self.looking_since = now

    def decision(self):
        """Stopped at a line: "stop_sign", or the light confirmed since stopping (the 2-of-3
        window can still hold the last frames before the stop), or "none".

        A newest red/yellow is a wait even before it confirms. Returning "none"
        here would let a course take its ordinary no-light continuation path.
        """
        if self.stop_sign:
            return "stop_sign"
        if self.looking_since is not None:
            hazard = self.newest_stop_light()
            if hazard is not None:
                return hazard
        if (self.state is not None and self.looking_since is not None
                and self.state_at >= self.looking_since):
            return self.state
        return "none"

    def green(self):
        """Stopped green permission, including newest-hazard and stop-sign priority."""
        return self.decision() == "green"

    def went_on(self):
        """The robot goes on across a line: start the next approach with the line armed."""
        self.passed = True
        self.state = self.state_at = self.looking_since = None
        self.stop_sign = False
        self.said = None

    def preview(self):
        """RV: what a front line would get from the lights confirmed right now."""
        light = self.light()
        if light in ("red", "yellow"):
            return "wait"
        if "stop_sign" in self.confirmed:
            return "end"
        return "cross" if light == "green" else "stop"


class VisionSession:
    def __init__(self, options=CameraOptions(), stream_factory=VisionStream):
        self.options, self.stream_factory = options, stream_factory
        self.enabled = False  # VT opt-in for the lane modes; never silently enables at startup.
        self.stream = None

    def close(self):
        if self.stream is not None and self.stream.close() is False:
            return False
        self.stream = None
        return True

    def start(self, control, debug=False):
        control.motors.stop(brake=False)
        control.poll()
        if not self.close():
            raise VisionFault("Previous camera worker is still blocked; restart before retrying vision")
        self.stream = (self.stream_factory(self.options, debug=True) if debug
                       else self.stream_factory(self.options))
        deadline = control.clock.monotonic() + VISION_START_TIMEOUT
        control.log("Starting Ryan's camera detector; motors disabled. 's' cancels.")
        while True:
            now = control.clock.monotonic()
            control.poll()
            sample = self.stream.latest()
            if sample is not None:
                return check_vision_sample(sample, now)
            if now >= deadline:
                raise VisionFault("Camera startup timed out; check CSI ribbon/device and dependencies")
            control.tick_sleep(now)

    def readings(self, control, debug=False):
        """RV; with ``debug`` (RVD, home testing), also why each colored blob is or is not a
        light, once a second, and 'p' saves a snapshot on the Pi."""
        try:
            self.start(control, debug)
            # RVD drives nothing, so it allows a slow frame while a snapshot is written.
            light = TrafficLight(self.stream, log=lambda text: None, max_age=RVD_MAX_AGE if debug else None)
            next_print = next_debug = 0.0
            if debug:
                control.log("RVD: every second, the frame's brightness and up to 3 red/yellow/green blobs with the "
                            "first of Ryan's checks each one fails, plus stop-sign rejection reasons. 'p' saves a snapshot (plain and annotated "
                            "JPEG in ~/rv_snapshots on the Pi).")
            control.log("RV: live detections only; motors disabled. 's' returns to menu.")
            control.log("Stop signs searched across the WHOLE frame; lights in TOP half after camera rotation.")
            control.log(f"A detection counts once 2 of 3 frames reach its action size: lights "
                        f"{settings.LIGHT_ACT_AREA_FRAC:.2%}, stop sign {settings.STOPSIGN_ACT_AREA_FRAC:.2%} "
                        "of frame (SET -> Camera). 'line' is what a front line would get: cross, wait "
                        "(red/yellow), end (stop sign) or stop (no light).")
            while True:
                now = control.clock.monotonic()
                keys = control.poll()
                if debug and "p" in keys:
                    self.stream.snap.set()
                    control.log("RVD: snapshot requested")
                light.update(now)
                while debug and self.stream.snapshots:
                    control.log("RVD snapshot saved: " + self.stream.snapshots.pop(0)
                                + " (and the same name without _annotated)")
                if debug and now >= next_debug and light.previous.debug:
                    for line in light.previous.debug:
                        control.log(line)
                    next_debug = now + 1.0
                if now >= next_print:
                    sample = light.previous
                    locations = {label: f" {position} center={center}" for label, position, center in sample.locations}
                    raw = ", ".join(f"{label}{locations.get(label, '')} ({area:.2%} frame)"
                                    for label, area in sample.detections) or "none"
                    confirmed = ", ".join(sorted(light.confirmed)) or "none"
                    control.log(f"Camera: {raw} | confirmed: {confirmed} | line: {light.preview()} "
                                f"| frame age {now-sample.timestamp:.2f}s | processing {sample.processing_seconds:.2f}s")
                    next_print = now + 0.5
                control.tick_sleep(now)
        finally:
            control.motors.stop()
            self.close()

    def attach(self, control, run):
        """Run ``run()`` with the camera on (VT): its light decides each front line.

        GO's countdown runs once the camera is up, so the clock starts with the wheels.
        """
        try:
            self.start(control)
            control.light = TrafficLight(self.stream, control.log)
            control.go_ready()
            return run()
        finally:
            control.motors.stop()
            control.light = None
            self.close()

    def forward(self, control):
        """FC with the camera (and ToF too, when tof_support calls this)."""
        return self.attach(control, lambda: control.run_test("FC"))
