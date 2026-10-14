# Local stop-sign changes, 2026-10-06

This update intentionally changes stop_sign_detector.py from the pinned source below: full-frame search, bright-red evidence required for white-core glare rejection, and optional diagnostics from the real rejection paths. Upstream geometry, size and other glare checks remain. See ../STOP_SIGN_UPDATE.md and test_stop_sign_update.py.

# Ryan rr source record for v9.3

Repository: https://github.com/ColinE0/2.09-prime

Branch: `rr`

Pinned commit: `31943865d2f9f7e9696ab8360a0f9e2cc6dc10f0` (updated 2026-09-27, Colin: "pull it in")

Subject: `Refactor traffic light detection thresholds and methods`

Author/time: reyezz1322 (Ryan Reyes), 2026-09-25 23:02:26 UTC. Brings in `f0f6aee`
(camera setup and vision data handling) and `884b6b8` (saturation and brightness
thresholds), both 2026-09-25.

2026-09-27 update from `1256a57`:
- `traffic_light_detector.py`: replaced with the exact blob at `3194386`
  (git blob `5bfba3bc`). Thresholds: MIN_SATURATION 100 -> 50; red and yellow
  bright ratio 0.35 -> 0.45; green gains its own candidate gate (S 80, V 180) and
  mean / peak / bright ratio 170 / 215 / 0.30 -> 180 / 220 / 0.45; RED1 hue upper
  15 -> 12. The precise bulb box now reuses the ROI's HSV and winning mask.
- `vision.py`: only `setup_camera` changed upstream (full-FOV 1640x1232 sensor mode
  scaled to 640x480, video configuration); ported into the standalone setup here.
  Detection and arbitration upstream changed comments only, so the adapted
  functions are unchanged. The menu camera (`vision_support`) already captured from
  the 1640x1232 mode, so menu frames are unaffected.
- `stop_sign_detector.py`: unchanged upstream.
- `../VERIFICATION.json` is the v9.4 snapshot and still names `1256a57`; this file
  is the current pin.

Previous pin: `1256a573380d4898c04537b883208109c806f0ed`, `Refine traffic light
detection thresholds and parameters`, reyezz1322 (Ryan Reyes), 2026-09-23 11:17:06 -0700.

This includes `492eaf8ea8511d548aea311eeeb1cb237babeeb2` (2026-09-23
11:14:01 -0700), `Enhance camera setup with rotation`. V9.2 previously bundled
`7183d7baf97e1ecb51d284eb8b3e4f881464d219` from September 19.

Fetched into a separate local checkout on 2026-09-23. No remote branches were
changed. Detector files were copied from Git blobs so Windows newline conversion
does not change the committed bytes.

## Source copies and local adaptations

- `stop_sign_detector.py`: exact Git blob; unchanged by today's upstream commits.
- `traffic_light_detector.py`: exact Git blob, including revised thresholds,
  brightness-weighted color scoring, nearby white-core handling, and compactness/
  aspect-ratio filtering. No thresholds overridden locally.
- `vision.py`: upstream file with package-relative detector imports, lazy
  Picamera2/libcamera imports inside `setup_camera`, `show_preview=False`, and
  removal of the unused GUI cleanup call. Navigation results and overlap
  arbitration are unchanged. Standalone setup retains Ryan's new 180-degree
  libcamera transform.
- `detector_geometric.py`: existing adapter retained. Calls `get_vision_data`
  with the menu worker's already rotated frame; it never calls upstream camera
  setup. It converts bounding-box size into full-frame area fractions, carries
  centers/position labels, and clears the upstream light cache before each frame
  so cached green results cannot masquerade as fresh confirmations.

## One camera rotation

The menu owns its camera through `vision_support.HeadlessCamera`, at 640x480
processed BGR, using its existing per-bot rotation/exposure settings. Rotation
occurs once in that camera reader before detection. The upstream standalone
camera setup is a separate entry point; its libcamera transform is never applied
to menu frames. `camera_rotate_180=False` / `--camera-upright` still work.

Stop signs use the right half between 5% and 80% height; lights use the top half.
Sign-over-overlapping-red arbitration is retained. FC action thresholds still
use bounding-box area divided by full frame area, so refined upstream boxes can
change when a detection becomes large enough to affect FC. Camera freshness,
red/yellow confirmation, green release and stop-sign termination remain in the
menu layer. Validate actual props in RV before enabling VT/FC.

The pinned rr tree has no LICENSE file. No unrelated license declaration was
added. See `../VERIFICATION.json` for source and packaged hashes and test results.
