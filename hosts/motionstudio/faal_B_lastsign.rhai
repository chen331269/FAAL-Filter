





// FAAL — reference translation for MotionStudio, before_builtin stage (api_version 1)
//
// This file is the canonical translation: JoyShockMapper's 2D filter path,
// nothing added. Every version that does player-space work should reuse this
// core unchanged.
//
// Placement: "内置滤波之前" (before_builtin) - the stage already supplies the
// horizontal / vertical aim rates with z = 0, exactly what JSM's 2D path filters.
//
// Budget
//   The engine allows 4096 Rhai operations per sensor sample for the whole chain
//   and 5 ms of wall clock. Building maps or re-reading ~20 params on every frame
//   exceeds that and the instance gets bypassed, so everything is read once on
//   the first call (MotionStudio recreates the instance whenever params change)
//   and the per-sample path is a fixed sequence of array accesses and arithmetic.
//
// State
//   Arrays instead of many named fields: fewer nodes, fewer operations per access.

fn clampv(v, lo, hi) {
    if v < lo { return lo; }
    if v > hi { return hi; }
    return v;
}

fn absv(v) {
    if v < 0.0 { return -v; }
    return v;
}

// Parameter lookup that accepts both spellings used in this project:
// short names in ms ("rise_ms") and JSM names in s ("FAAL_HYST_RISE_TIME").
// Only called during init, never per sample.
fn pms(params, shortKey, jsmKey, fallbackMs) {
    if shortKey != "" && params.contains(shortKey) { return params[shortKey] * 1.0; }
    if jsmKey != "" && params.contains(jsmKey) { return params[jsmKey] * 1000.0; }
    return fallbackMs;
}

fn pf(params, shortKey, jsmKey, fallback) {
    if shortKey != "" && params.contains(shortKey) { return params[shortKey] * 1.0; }
    if jsmKey != "" && params.contains(jsmKey) { return params[jsmKey] * 1.0; }
    return fallback;
}

// One axis of the filter.
//   st : 5-element state [peak, |v|, freeze, hyst, lastSign]
//   k  : shared constants, see the layout table in process()
// Returns [out, peak, |v|, freeze, hyst, lastSign].
// cos() for the reversal angle. The host exposes exp/sqrt but its cos is not
// guaranteed, and a missing function aborts every frame, so approximate:
// 6th-order Taylor on [0, pi/2], mirrored for the rest. Error < 1e-3.
fn cos_approx(x) {
    let pi = 3.14159265358979;
    let a = absv(x);
    if a > pi { a = pi; }
    let s = 1.0;
    if a > 1.5707963267949 { a = pi - a; s = -1.0; }
    let a2 = a * a;
    return s * (1.0 - a2 * (0.5 - a2 * (0.0416666667 - a2 * 0.0013888889)));
}

fn axis(v, dt, st, k) {
    let a = absv(v);
    let peak = st[0];
    let lastSign = st[4];

    // 1) drop rate, normalised to 1 kHz and to the current peak
    let drop = 0.0;
    if peak > k[0] {
        let d = st[1] - a;
        if d > 0.0 { drop = d / (dt * 1000.0) / (peak + 0.001); }
    }

    // 2) frame-rate independent hysteresis coefficients
    let rise = 1.0 - exp(-dt / k[1]);
    let fast = 1.0 - exp(-dt / k[2]);
    let slow = 1.0 - exp(-dt / k[3]);
    let hyst = st[3];
    let fall = clampv(fast + (slow - fast) * hyst, 0.0, 1.0);

    // 3) peak tracking
    if a > peak { peak = a; }
    else { peak = peak * exp(-dt / k[4]); }


    // 5) drop-slope suppression: curve 1 = linear, curve 2 = the original
    //    quadratic. Only these two are implemented (no pow() in the host).
    let raw = 0.0;
    if peak > k[0] {
        let d = clampv(drop * k[5], 0.0, 1.0);
        raw = d;
        if k[11] > 1.5 { raw = d * d; }
    }

    // 6) Both change detectors, enabled together, full strength - which is how
    //    it was always used and never caused trouble. Neither is a "high speed"
    //    detector: what actually gets suppressed is the settling right after a
    //    flick, and there the angular speed is already low.
    // Compare against the last non-zero sample: a crossing that lands exactly
    // on zero is no longer swallowed, which removes the one-sided trigger
    // asymmetry on repeated strokes.
    let event = false;
    if k[6] > 0.0 && peak > k[0] && absv(v) > 0.000001 {
        if absv(lastSign) > 0.000001 && ((v < 0.0) != (lastSign < 0.0)) {
            if absv(v - lastSign) / (dt * 1000.0) > k[6] { raw = 1.0; hyst = 1.0; event = true; }
        }
        lastSign = v;
    }
    // (duplicate one-liner removed: the same override is applied below)

    // 7) vector-reversal override (flag prepared by the caller)
    if k[8] > 0.5 {
        raw = 1.0;
        hyst = 1.0;
        event = true;
    }

    // 8) hysteresis
    if raw > hyst { hyst = hyst * (1.0 - rise) + raw * rise; }
    else { hyst = hyst * (1.0 - fall) + raw * fall; }
    hyst = clampv(hyst, 0.0, 1.0);

    // 9) no smoothing stage: FAAL suppresses, a separate filter smooths. The
    //    One Euro module used to live here; it was stripped out on 2026-11-15
    let out = v * (1.0 - hyst);

    // 10) freeze window. Only a crossing or a reversal may *start* it - drop is a
    //     continuous level and must not clamp every deceleration. But once it is
    //     running, hyst keeps extending it, so a deeper or longer suppression
    //     (drop included) delays the release by keeping hyst above the threshold.
    let frz = st[2];
    if k[7] > 0.0 {
        if event && hyst > k[9] { frz = k[7]; }
        if frz > 0.0 {
            if hyst > k[9] { frz = k[7]; }
            out = out * k[10];
            frz = frz - dt;
            if frz < 0.0 { frz = 0.0; }
        }
    }

    return [out, peak, a, frz, hyst, lastSign];
}

fn process(sample, params) {
    // ------------------------------------------------------------------ init
    if !this.contains("ok") {
        this.ok = true;

        // per-axis state, 5 values each
        this.S = [0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0];
        // reversal reference
        this.D = [0.0, 0.0];
        // duplicate-report accumulator
        this.acc = 0.0;
        // last input/output, for duplicate merging
        this.lx = 1e9;
        this.ly = 1e9;
        this.lz = 1e9;
        this.lout = [0.0, 0.0, 0.0];

        let fall = clampv(pms(params, "fall_ms", "FAAL_HYST_FALL_TIME", 65.0) / 1000.0, 0.0001, 1.0);
        let ff = pms(params, "fall_fast_ms", "FAAL_HYST_FALL_FAST_TIME", 0.0);
        let fastFall = fall * 0.446;
        if ff > 0.0 { fastFall = clampv(ff / 1000.0, 0.0001, fall); }

        // K layout
        //  0 peak_min        1 rise        2 fastFall    3 fall       4 peak_decay
        //  5 drop_sens      6 sign_thr    7 freeze      8 force (per frame)
        this.K = [
            pf(params, "peak_min", "", 2.2),
            clampv(pms(params, "rise_ms", "FAAL_HYST_RISE_TIME", 2.9) / 1000.0, 0.0001, 1.0),
            fastFall,
            fall,
            clampv(pms(params, "peak_decay_ms", "FAAL_PEAK_DECAY_TIME", 65.0) / 1000.0, 0.0001, 1.0),
            pf(params, "drop_sens", "FAAL_DROP_SENSITIVITY", 2.4),
            pf(params, "sign_thr", "FAAL_SIGN_CHANGE_THR", 0.5),
            clampv(pms(params, "freeze_ms", "FAAL_FREEZE_DURATION_SEC", 24.0) / 1000.0, 0.0, 1.0),
            0.0,
            pf(params, "suppress_thr", "", 0.45),
            clampv(pf(params, "freeze_mul", "", 0.1), 0.0, 1.0),
            pf(params, "drop_curve", "", 2.0),
        ];

        this.smoothTime = clampv(pms(params, "smooth_time_ms", "FAAL_SMOOTH_TIME_SEC", 7.5) / 1000.0, 0.001, 1.0);
        // The One Euro module (fc_min / beta / speed low-pass) was stripped on
        // 2026-11-15: FAAL now suppresses only, and smoothing is a separate stage.
        this.revOn = pf(params, "rev_on", "FAAL_ENABLE_VECTOR_REVERSAL", 1.0);
        // an explicit cos wins; otherwise take the angle the profile carries
        let rc = -2.0;
        if params.contains("rev_cos") { rc = params["rev_cos"] * 1.0; }
        this.revCos = if rc > -1.5 {
            clampv(rc, -1.0, 1.0)
        } else {
            clampv(cos_approx(pf(params, "rev_angle_rad", "FAAL_DIR_CHANGE_ANGLE_RAD", 1.2)), -1.0, 1.0)
        };
        this.revMag = clampv(pf(params, "rev_mag", "FAAL_DIR_CHANGE_MAG_RATIO", 0.15), 0.0, 1.0);
        this.revFloor = clampv(pf(params, "rev_min_dps", "", 5.0), 0.0, 100.0);
        this.dedup = pf(params, "dedup", "", 1.0);
        this.dtFloor = pms(params, "dt_floor_ms", "", 0.05) / 1000.0;
        if this.dtFloor <= 0.0 { this.dtFloor = 0.00005; }
        // Real report interval. report_ms > 0 pins it; otherwise it is estimated
        // from the gaps the host reports between genuinely new readings.
        this.report = clampv(pms(params, "report_ms", "", 0.0) / 1000.0, 0.0, 0.05);
        this.period = 0.0;
        // Worst case per sample decides whether the whole chain fits the host's
        // operation budget. Two steps keeps it small; raise it only if the host
        // has budget to spare.
        this.maxSteps = pf(params, "max_steps", "", 1.0);
        if this.maxSteps < 1.0 { this.maxSteps = 1.0; }
        if this.maxSteps > 8.0 { this.maxSteps = 8.0; }
    }

    // -------------------------------------------------------------- per sample
    //
    // The filter keeps its own clock. Incoming dt is accumulated and spent in
    // whole milliseconds, so the host's tick rate decides only how fresh the input
    // is - never how the filter advances. JoyShockMapper gets this for free by
    // polling at a fixed rate; this gives the same guarantee without the host's
    // cooperation, which is what makes the tuned time constants hold on any engine
    // rate.
    //
    // It pairs with the sensor timestamp option: that one supplies an accurate dt
    // (including dt = 0 for repeated reports), and this turns those chunks into a
    // steady 1 ms cadence instead of one big irregular step.
    // Advance only on a genuinely new reading. Repeated reports carry no time of
    // their own, and a host that repeats a nonzero dt for them must not be able to
    // run the filter clock fast.
    if sample.x == this.lx && sample.y == this.ly && sample.z == this.lz {
        return this.lout;
    }

    let dt = sample.dt;
    if dt < 0.0 { dt = 0.0; }
    if dt > 0.05 { dt = 0.05; }

    // A pinned interval wins; otherwise estimate it, so a jittery host clock still
    // advances the filter by the true average period.
    if this.report > 0.0 {
        dt = this.report;
    } else if dt > 0.0 {
        if this.period <= 0.0 { this.period = dt; }
        else if dt > this.period * 3.0 || dt < this.period * 0.34 {
            // A real gap or a fresh rate: follow it rather than lag.
            this.period = dt;
        } else {
            this.period = this.period + 0.2 * (dt - this.period);
        }
        dt = this.period;
    }

    this.acc = this.acc + dt;
    if this.acc < 0.001 {
        return this.lout;
    }

    // Axis mapping per stage. raw_gyro is canonical X forward / Y right / Z down,
    // so yaw is Z and pitch is Y. The aiming stages already are yaw / pitch.
    let three = sample.components > 2;
    let roll = 0.0;
    let yaw = sample.x;
    let pitch = sample.y;
    if three {
        roll = sample.x;
        pitch = sample.y;
        yaw = sample.z;
    }

    let oyaw = this.lout[0];
    let opitch = this.lout[1];
    let steps = 0;

    // Four 1 ms steps cover a 250 Hz pad report; six leaves headroom for a slower
    // pad without risking the per-sample operation budget.
    while this.acc >= 0.001 && steps < this.maxSteps {
        // Spend it in one step when the remaining steps could not cover it: the
        // per-sample operation budget is finite and an overrun bypasses the whole
        // filter. This keeps the cost at maxSteps axis-pairs per sample.
        let big = this.acc > (this.maxSteps - steps + 1.0) * 0.001;
        let step = if big { this.acc } else { 0.001 };
        this.acc = if big { 0.0 } else { this.acc - 0.001 };
        steps = steps + 1;

        let K = this.K;
        let speed = sqrt(yaw * yaw + pitch * pitch);
        K[8] = 0.0;

        // vector reversal, judged before this sample is absorbed
        let alpha = 1.0 - exp(-step / this.smoothTime);
        if this.revOn > 0.5 {
            let pl = sqrt(this.D[0] * this.D[0] + this.D[1] * this.D[1]);
            // below a few dps the direction is just noise: without this floor the
            // detector fires every few ms at rest and each fire arms a freeze
            if pl > this.revFloor && speed > this.revFloor {
                let c = clampv((this.D[0] * yaw + this.D[1] * pitch) / (pl * speed), -1.0, 1.0);
                if c < this.revCos {
                    // the reference lags the hand, so require the new motion to be
                    // a real fraction of it. |sin| was the old gate and it vanishes
                    // at a clean 180 deg reversal - the exact case this is for.
                    if speed > this.revMag * pl {
                        K[8] = 1.0;
                        // one trigger per reversal: absorb the new direction so
                        // the lagging reference cannot re-fire every step
                        this.D[0] = yaw;
                        this.D[1] = pitch;
                    }
                }
            }
        }
        this.D[0] = this.D[0] + alpha * (yaw - this.D[0]);
        this.D[1] = this.D[1] + alpha * (pitch - this.D[1]);
        this.K = K;

        let S = this.S;
        let rx = axis(yaw, step, [S[0], S[1], S[2], S[3], S[4]], K);
        let ry = axis(pitch, step, [S[5], S[6], S[7], S[8], S[9]], K);

        this.S = [rx[1], rx[2], rx[3], rx[4], rx[5],
                  ry[1], ry[2], ry[3], ry[4], ry[5]];

        oyaw = rx[0];
        opitch = ry[0];
        if oyaw != oyaw || opitch != opitch {
            this.ok = false;
            return [sample.x, sample.y, sample.z];
        }
    }

    this.lx = sample.x;
    this.ly = sample.y;
    this.lz = sample.z;

    if three {
        // keep roll untouched, return pitch on Y and yaw on Z as the stage expects
        this.lout = [roll, opitch, oyaw];
    } else {
        this.lout = [oyaw, opitch, sample.z];
    }
    return this.lout;
}
