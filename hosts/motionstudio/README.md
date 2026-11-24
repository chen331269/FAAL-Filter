# FAAL as a MotionStudio custom motion filter

Four Rhai scripts that run FAAL inside MotionStudio, plus the tooling that keeps
them honest. They are the host-side counterpart to `faal.h`: same suppression
algorithm, ported to the engine's 1 ms internal clock.

## Which script goes where

| File | Stage | Use |
|---|---|---|
| `faal_B_rev_mag_v10.rhai` | `before_builtin` (2D) | **current baseline**: the turn/pitch pair, projected by the host |
| `faal_B_lastsign_v1.rhai` | `before_builtin` (2D) | the previous mainline: `\|sin\|` reversal only, no freeze gate; kept for comparison |
| `faal_ps_v2.rhai` | `raw_gyro` (3D) | player-space path with its own gravity estimate and PLAYER gain |
| `faal_ps_world_v1.rhai` | `raw_gyro` (3D) | pure world-space (AxisRay) projection, for hosts that would otherwise apply their own space conversion |

Only one of them belongs in a profile at a time. `faal_B_rev_mag_v10.rhai` is the
one in current play; `faal_B_lastsign_v1.rhai` is the version before the freeze
gate and is kept so the two can be compared on the same signal.

## What the filter does now

**Suppression only.** The One Euro smoothing stage was removed on 2026-11-15:
smoothing belongs downstream where its latency is a separate, visible choice.
See the repo README for why.

Per axis, six state values are carried:

```
[peak, |v|, freeze, hyst, lastSign]
```

and the output is

```
out = v × (1 − hyst)          × freeze_mul while the freeze window is active
```

- `raw` (the target suppression) is the quadratic drop rate, or 1 on a sign
  crossing / vector reversal;
- `hyst` chases `raw` with a 2.9 ms rise and a 65 ms fall (the fall shortens to
  ~29 ms once `hyst` is low);
- the freeze window is *started* only by a sign crossing or a vector reversal,
  never by the drop level (drop is continuous, so starting on it clamps every
  deceleration). While it runs, `hyst` keeps extending it, so a deeper or longer
  suppression delays the release; when `hyst` falls back below `suppress_thr` the
  window drains in `freeze_ms`. Output is multiplied by `freeze_mul` (default 0.1).

## Parameters

Short names are what the script reads; the `FAAL_*` names are the JSM-style
fallbacks, and `params` accepts either.

| Short | JSM key | Default | Meaning |
|---|---|---|---|
| `peak_min` | — | 2.2 | deg/s below which nothing counts as motion |
| `rise_ms` | `FAAL_HYST_RISE_TIME` | 2.9 | how fast suppression clamps down |
| `fall_ms` | `FAAL_HYST_FALL_TIME` | 65 | how slowly it lets go |
| `fall_fast_ms` | `FAAL_HYST_FALL_FAST_TIME` | 0 | overrides the low-`hyst` release (0 = 0.446 × `fall_ms`) |
| `peak_decay_ms` | `FAAL_PEAK_DECAY_TIME` | 65 | peak tracker decay |
| `drop_sens` | — | 1.0 | drop-rate → suppression gain |
| `sign_thr` | `FAAL_SIGN_CHANGE_THR` | 0.5 | deg/s per ms that counts as a crossing |
| `drop_curve` | — | 1.0 | drop shaping: 1 = linear, 2 = the original quadratic |
| `freeze_ms` | `FAAL_FREEZE_DURATION_SEC` | 24 | freeze window length |
| `suppress_thr` | — | 0.45 | `hyst` level that opens the freeze window |
| `freeze_mul` | — | 0.1 | how much leaks through while frozen |
| `rev_on` | `FAAL_ENABLE_VECTOR_REVERSAL` | 1 | vector-reversal override |
| `rev_angle_rad` | `FAAL_DIR_CHANGE_ANGLE_RAD` | 1.2 | how sharp a turn counts as a reversal (69°); `rev_cos` still wins if present |
| `rev_mag` | `FAAL_DIR_CHANGE_MAG_RATIO` | 0.15 | with `sign_thr > 0`: the `\|sin θ\|` gate; with `sign_thr = 0`: the velocity-ratio gate and must not be 0 |
| `rev_freeze_dps` | — | 90 | `sign_thr = 0` only: reference speed below which a trigger only dips, without arming the freeze |
| `smooth_time_ms` | `FAAL_SMOOTH_TIME_SEC` | 7.5 | only the reversal reference's low-pass |

`FAAL_ONE_EURO_FC_MIN` / `FAAL_ONE_EURO_BETA` are no longer read; delete them
from a profile that still carries them.

## Importing

Paste the script into the motion filter's source box, or write it straight into
the profile with `inject.pl` while MotionStudio is closed (it also mirrors
`.bak` and keeps a timestamped rollback):

```
perl inject.pl faal_B_rev_mag_v10.rhai
perl inject.pl faal_B_rev_mag_v10.rhai --only 0   # default layer only
perl inject.pl faal_B_rev_mag_v10.rhai --dry
```

## Keeping the core in step

`faal_B_lastsign_v1.rhai` is the reference for the FAAL core. The other scripts
are deliberately outside the hash check: `faal_B_rev_mag_v10.rhai` adds the
freeze-gate slot and splits the reversal detector, and the `faal_ps_*` scripts
run the same core on a 3D path. Edit the core in the reference, then:

```
python check_core.py          # hash + structural + index checks
```

The structural check is the one that matters: the per-axis state width is
duplicated in six places (init, the axis calls, both store-backs, both
returns), and out-of-range indices bypass the whole filter at runtime with a
single "bypassed" message in the UI.

Current core hash: **`62cbf6919484`**.

## Where `faal.h` stands

The C library was frozen at the v4 semantics on 2026-11-18 (`f8204e9`). It has
the same two-mode reversal detector, but three differences matter if you are
porting between the two:

| | `faal.h` (v4) | `faal_B_rev_mag_v10.rhai` |
|---|---|---|
| reference absorption | always absorbs, so the `\|sin\|` branch fires once | the `\|sin\|` branch does **not** absorb: it re-fires while the reference catches up |
| startup guard | none | the first 50 ms may suppress but may not arm the freeze |
| `revMinDps` | still present | removed on 2026-11-18 |

So v10 currently lives only in the Rhai host scripts; the guard and the
no-absorb branch are the two things to port if the C path has to match.

## Versioning

Every change bumps `_vN` in the file name, so the script you imported can be
identified at a glance: `faal_B_rev_mag_v10.rhai` is newer than `_v9`. Each file
opens with its version, date and one line of what changed. `check_core.py`
resolves the highest `_vN` per prefix, so a bump never needs a code change here.

## Milestone: v4 (2026-11-18, superseded by v10)

`faal_B_rev_mag_v4.rhai` is the first version that felt right in play. Its
vector detector does three things at once, and that combination is the whole
point:

* it compares the current velocity with a 7.5 ms low-passed reference direction;
* when the angle clears `rev_angle_rad` and the perpendicular part clears
  `rev_mag`, it forces `hyst = 1` and pulls the reference onto the new direction,
  so one reversal yields exactly one trigger;
* because an incoherent direction (rest noise) trips it constantly while a
  coherent one (a slow micro-adjustment, a circle) does not, it doubles as a
  *smart dead zone*: the crosshair pins by itself, yet nothing intentional is
  eaten.

Two modes are chosen once at init from `sign_thr`:

| `sign_thr` | gate | who owns a clean 180 deg reversal |
|---|---|---|
| > 0 | `\|sin θ\| > rev_mag` | the sign detector; this one only reinforces |
| 0 | `\|v\| > rev_mag × \|ref\|` | this one |

With `sign_thr = 0` the freeze is additionally gated: a trigger whose reference
is slower than `rev_freeze_dps` only dips, it does not arm the 24 ms window.

### Shipped parameters (v4, 2026-11-18)

| Parameter | default | ADS layer | third layer |
|---|---|---|---|
| `FAAL_DIR_CHANGE_ANGLE_RAD` | 0.9 | 0.9 | 0.9 |
| `FAAL_DIR_CHANGE_MAG_RATIO` | 0.05 | 0.05 | 0.05 |
| `FAAL_SIGN_CHANGE_THR` | 0.5 | 0 | 0 |
| `FAAL_ENABLE_VECTOR_REVERSAL` | 1 | 1 | 0 |
| `FAAL_DROP_SENSITIVITY` | 1.0 | 1.0 | 2.4 |
| `drop_curve` | 1.0 | 1.0 | 2.0 |
| `rev_freeze_dps` | 90 | 90 | — |
| `FAAL_FREEZE_DURATION_SEC` | 0.024 | 0.024 | 0.024 |
| `freeze_mul` | 0.1 | 0.1 | 0.1 |
| `FAAL_HYST_RISE_TIME` | 0.0029 | 0.0029 | 0.0029 |
| `FAAL_HYST_FALL_TIME` | 0.065 | 0.065 | 0.065 |
| `FAAL_PEAK_DECAY_TIME` | 0.065 | 0.065 | 0.065 |

## Milestone: v10 (2026-11-24)

v10 keeps v4's freeze gate but splits the reversal detector back into the two
behaviours it was merging. The mode is fixed at init from `sign_thr` (the engine
rebuilds the instance whenever a parameter changes, so switching layers switches
modes with it):

| | sign detector on (`sign_thr > 0`) | sign detector off (`sign_thr = 0`) |
|---|---|---|
| gate | `sqrt(1 − c²) > rev_mag`, i.e. `\|sin θ\|` | `speed > rev_mag × \|ref\|` |
| reference absorption | none — it re-fires while the reference catches up, which is where the older mainline's punch came from | absorbed, so one reversal yields one trigger |
| freeze gate `rev_freeze_dps` | inactive | active: a slow reference only dips, it does not freeze |
| clean 180° reversal | left to the sign detector | caught here |

Both modes carry the same startup guard: for the first 50 ms (`nStep <= 50`) a
trigger may only suppress, it may not arm the freeze window.

The reason for the split: with the sign detector running, v4's absorption made
the turn feel weaker than the pre-v4 mainline had, while with the detector off
nothing else can see a clean 180° and the magnitude gate plus its freeze gate are
the only way to catch it. v10 is v4 with that split and the startup guard.

