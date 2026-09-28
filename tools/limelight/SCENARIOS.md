# Numbered Limelight 3A capture guide

These are proposed bench/practice-field test positions, not official game dimensions
or validated shooting positions. Start with S01–S04; add the remaining tests as the
setup allows. Each table row variant is a separate recording. The recorder saves
detection JSON only and does not automatically walk through these instructions.

## Set up the reference positions

1. Use a real cell in its stable upward-facing position when available. Otherwise,
   use a rigid tag board with correct tag sizes, IDs, spacing and decoded orientation.
   An arbitrary wall arrangement is useful for detection tests only: mark
   `geometry=wall-only` and `--expected-state unknown`; do not treat its calculated
   opening position as ground truth. These sessions are not ChArUco lens calibration.
2. Choose one cell, initially RED AUDIENCE. Check its IDs: red audience 34–37,
   red scoring-table 30–33, blue audience 38–41, blue scoring-table 42–45.
3. Mark **C** on the floor directly below the center of the opening. The centerline
   extends from C toward the camera's shooting side, perpendicular to the opening's
   left/right axis. Measure the opening height **H** above the floor.
4. Positions below are **(L, D)** in meters. D is floor distance out along that
   centerline. L is sideways displacement: positive to the right when standing on
   the camera side and looking toward the cell. Mark left/right directions on the
   floor so the convention stays consistent when you change sides.
5. Set the camera lens center at **h = 0.40 m** above the floor for the baseline.
   This is a repeatable starting height, not a mount recommendation. If it is
   obstructed or unsuitable, choose another height and record that value on every take.
   Use a stand for stationary tests; don't score hand tremor as camera jitter.
6. At (0, 1.50), aim toward the opening, then adjust pitch enough to keep the tag
   cluster visible. Keep camera roll zero. Record pitch if you can measure it.
   Exact tag-plane head-on alignment is not required; preserve the actual geometry.
7. Check the live preview and recorder: the expected IDs should appear, with corner
   output available. Keep pipeline, resolution, crop/zoom, calibration and exposure
   fixed for a set of takes. Repeat selected takes after changing one setting.

**D is not the recorder's `--distance`.** That option is the straight-line lens-to-
opening distance. Measure it independently, or calculate
`sqrt(L*L + D*D + (H-h)*(H-h))` from measured coordinates. Do not pass floor distance
or the Limelight's own estimated distance. Omit it whenever camera/cell motion changes
the distance, or the opening reference is not known.

If a mark is blocked by field structure, cannot be reached, or puts the target out
of view, record the actual position or label the take as an unavailable position.
Do not quietly change height/distance to make a failure disappear.

## Run each take

This example records S01. Replace the scenario name, alliance, side, state, duration
and notes for the row you are performing. `--pipeline-index 0` should match the
pipeline already selected in the camera UI; it does not switch pipelines.

```sh
python3 tools/limelight/record.py record \
  --scenario red-audience-S01-front-take1 \
  --alliance red --side audience --expected-state upright \
  --pipeline-index 0 --duration 10 \
  --notes "geometry=real-cell; L=0m D=1.50m h=0.40m; camera stationary; default calibration"
```

Position the camera **before** launching a timed take; capture starts after the
configuration reads. Use the first `0.0s` status line as the start of motion timing.
If working alone, omit `--duration`, press Enter when ready, perform the sequence,
then Ctrl+C. Approximate timings are fine if deviations are noted. For stationary
takes, add `--distance YOUR_MEASURED_SLANT_DISTANCE` with an actual number.

`--expected-state` describes the **cell**, not the camera. A camera moving past a
stationary upright cell still uses `upright`. `moving` means the cell itself is
moving; `none` means no relevant target is visible. These labels document intent,
not automated pass/fail assertions.

## Stationary tests

Use 10 seconds per take, camera roll zero. Repeat S01–S04 for red and blue. If the
field is assembled, repeat on both audience and scoring-table sides; change the
CLI `--side` to match. Begin with one take each, then make three takes for settings
you want to compare quantitatively.

| Test | Camera position / action | What to check |
| --- | --- | --- |
| **S01 — Front baseline** | (0, 1.50), h=0.40. Aim toward the opening; hold still. | Correct IDs/side, corner availability, stable opening angles and measured range. Angles need not be zero if the optical axis is not exactly on the opening. |
| **S02 — Near and far** | Separate takes at (0, 0.75), (0, 1.50), (0, 2.50). Keep height fixed and re-aim toward the opening. | Range error vs independently measured slant distance, repeatability, tags lost at distance. If all tags do not fit at the near position, preserve that result. |
| **S03 — Left/right oblique** | At h=0.40: (-0.75, 1.30), (+0.75, 1.30), then (-1.06, 1.06), (+1.06, 1.06). Aim toward the opening at every mark. These are about 30° and 45° off the centerline, at roughly 1.50 m floor radius. | Pose flips, angle/range consistency, partial visibility. Record four separate takes. These angles describe floor viewing direction, not camera yaw Euler output. |
| **S04 — Off-center image** | At (0, 1.50), do four takes: yaw camera 10° left/right from baseline; then pitch it 8° up/down from baseline. Return to baseline before each change. Keep position and roll fixed. | Turning camera left should shift target X right; pitching up should shift target Y down. Check opening offsets, not the center of an individual tag. Preserve loss-of-view if it occurs. |
| **S05 — Mount height** | At (0, 1.50), separate takes with lens h=0.25, 0.40 and 0.60. Re-aim to see the cluster; record pitch. | Whether candidate mounting heights see enough tags without obstructing the opening view; consistency of 3D position. Substitute actual proposed robot heights if known. |
| **S06 — Partial tags** | Baseline pose. Separate 10-second takes showing four tags, then two adjacent tags, then one tag. Cover complete individual tags with plain opaque cards without moving the cell. Record which IDs remain. Repeat one-tag take using the other end tag. | Accuracy and continuity with partial visibility; one tag may be more ambiguous. Leave full white borders visible around each uncovered tag. |
| **S07 — Inverted cell** | Baseline position facing the selected cell after it reaches its opposite stable state in an off-match setup. Keep recording that same alliance/side. Use `--expected-state inverted`. | The selected inverted cell should not produce an aiming candidate, even if tags remain visible. If tags disappear, this tests missing data rather than orientation rejection. |
| **S08 — Competing targets** | On a full field, start at baseline and shift sideways only as needed to see another cell. Record actual L/D. First select your actual alliance/side; then repeat while looking only at opponent tags, without changing `--alliance`. | Correct filtering. Opponent-only take should return no candidate. For a two-side take, also replay/record once with `--side either` to examine nearest-candidate selection. |

S07 on a loose board rotated 180° in its plane is a useful inversion sanity check,
but does not reproduce the real hive's tipping motion or changed position. Label it
as a board test. Do not extrapolate its result to full-field shootability.

## Moving and recovery tests

Keep the cell upright for S09–S11. Record `--expected-state upright` and omit a
single `--distance`. Camera movement should change measured position; this is not
a stationary jitter test. A swivel/stand gives more repeatable rotation than handholding.

| Test | Camera position / timed action | What to check |
| --- | --- | --- |
| **S09 — Slow yaw sweep** | Baseline lens position. 20 s: hold 5 s; yaw from 15° left of baseline to 15° right over 5 s; return over 5 s; hold 5 s. Note that the move to the starting angle occurs at the end of the initial hold. | Continuous detection, aiming-sign changes, reacquisition near image edges. Camera height/translation should stay approximately fixed. |
| **S10 — Fast yaw sweep** | Same setup, 20 s: hold 5 s; sweep between ±15° approximately once per second for 10 s; hold baseline for 5 s. | Motion-related dropouts and corner-fit degradation. Compare different exposure settings using separate sessions. |
| **S11 — Translate / approach** | Two 20 s takes. **Lateral:** 5 s at (-0.50, 1.50), move to (+0.50, 1.50) over 10 s, hold 5 s, keeping cluster in view. **Approach:** 5 s at (0, 2.50), approach (0, 0.75) over 10 s, hold 5 s. Keep h fixed as practical; note handheld motion. | Range trend, visibility changes and recovery after motion. Repeat in reverse if comparison is useful. |
| **S12 — Cell transition** | Camera stationary at baseline. In an off-match/practice setup, record 5 s stable, one hive transition, and at least 5 s after settling. Use `--expected-state moving`, 20–30 s total, and note approximate transition times. | Inversion/state behavior and pose discontinuities. The current class does not detect settling: a transitional candidate is not permission to fire. |
| **S13 — Lost view** | Camera and cell stationary. 15 s: clear view 5 s, cover lens 5 s, clear view 5 s. Use `--expected-state unknown` and put that sequence in notes. | No-target periods and reacquisition; no retained candidate from before the cover. Covering the lens does not necessarily stop fresh camera frames. |
| **S14 — Disconnect** | Baseline pose. Run without `--duration`; capture 5 s, disconnect USB for 5 s, reconnect, wait for camera to boot and detections to resume, then capture another 5 s and Ctrl+C. Use `--expected-state unknown`. | Read errors, timestamp reset, recovery. Boot time is additional; do not terminate after only 15 s. |

Once a turret exists, repeat S09/S10 with actual turret motion and S11 with actual
chassis motion. This recording contains no turret encoder/robot motion data, so it
can expose tracking problems but cannot validate compensation against measured
mechanism angles or velocities by itself.

## Lighting and calibration comparisons

For each lighting or exposure/gain configuration, repeat **S01, far S02, one 45°
S03, and S10**. Put lighting description, camera settings and calibration state in
the notes, and save the corresponding pipeline snapshot. Change only one variable
at a time. Use normal venue/practice lighting and any genuinely representative dim
or glare conditions; do not compare settings against different camera paths unnoticed.

For before/after lens calibration comparisons, repeat stationary tests at the same
floor marks, heights and measured opening location. A reduction in jitter alone
does not establish accuracy: compare against measured range too.

After every take, check `summary.json`: successful polls, observed new frames,
detected IDs, missing-corner polls and errors. A run with no corners is still useful
for detection diagnostics, but cannot exercise our corner-based opening solver.
The [replay instructions](README.md#replay-through-our-java-targeting-class) explain
how to generate opening X/Y and distance outputs once calibration and corner order
are verified. Keep failed takes; they are often the most useful regression data.
