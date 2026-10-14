# v9.3: Ryan's September 23 vision update

Built on v9.2 with Ryan's `rr` branch pinned at
[`1256a57`](https://github.com/ColinE0/2.09-prime/commit/1256a573380d4898c04537b883208109c806f0ed).
The preceding commit
[`492eaf8`](https://github.com/ColinE0/2.09-prime/commit/492eaf8ea8511d548aea311eeeb1cb237babeeb2)
adds standalone camera rotation. Both were authored September 23, 2026.

## Detector changes

The traffic-light module is copied exactly from the committed source. It changes
color/brightness thresholds and bounding-box refinement, associates bright white
core pixels with nearby color, weights competing colors by brightness, and
rejects elongated or irregular candidates. These are algorithm changes, not a
claim of measured accuracy on the robots. The stop-sign detector is unchanged.

The menu's camera already supports 180-degree rotation. It continues to rotate
frames once before detection; it does not call Ryan's standalone camera setup.
Per-bot upright/rotated and exposure choices remain effective. Upstream camera
imports are lazy so offline tests and non-camera commands still work off the Pi.

The existing adapter and FC response rules are retained, including fresh-frame
checks, cache clearing, immediate red/yellow holds, confirmation before latching,
and green-frame confirmation before release. New bounding boxes can change the
FC area threshold crossing distance. Use RV with the actual lights/sign before
turning camera responses ON with VT.

## Existing fixes and upgrading

The v9.2 automatic application of SET changes, individual-wheel reversal
shortcuts, INA reader ownership/recovery, and `--check-ina` diagnostic remain.
Both bot profiles remain separate. V9.3 first looks for each bot's tagged
profile in sibling `MPU6050_Motor_Test_v9_2/profiles/`, then v9.1 and v9. Existing
v9.3 profiles take precedence; old files are copied, never modified.

No source was pushed to GitHub. Earlier ZIPs and local packages are preserved.
Testing is offline with simulated camera/GPIO/I2C and synthetic images, not a
physical robot or Pi camera run. See TEST_RESULTS.txt, VERIFICATION.json and
[ryan_vision/SOURCE.md](ryan_vision/SOURCE.md) for verification and exact adaptations.
