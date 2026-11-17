// FAAL for MotionStudio — raw_gyro stage, player-space filter (PS_MOTION path).
//
// JoyShockMapper's PS_MOTION path does not filter device axes. It derives the
// gravity-referenced player-space pair, filters that pair, and then rebuilds a
// 3D vector. This script follows the same path:
//
//   1. up    = vertical, taken from the accelerometer
//   2. fwd   = device forward (X) projected onto the horizontal plane
//   3. right = up x fwd
//   4. yaw = g·up,  pitch = g·right,  roll = g·fwd
//   5. filter yaw and pitch with the FAAL core
//   6. out = yaw'·up + pitch'·right + roll·fwd
//
// Because the basis is derived from gravity instead of assumed axis roles, it
// works no matter how the device's axes are oriented, and the forward mesh keeps
// its direction: only the filtered components change.
//
// dt is used exactly as delivered (no merging, no flooring).

fn clampv(v, lo, hi) {
    if v < lo { return lo; }
    if v > hi { return hi; }
    return v;
}

fn absv(v) {
    if v < 0.0 { return -v; }
    return v;
}

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

// One axis of the FAAL core.
//   st : [peak, prevAbs, prevV, still, freeze, hyst, oneEuro]
//   k  : shared constants, see the layout table in process()
// Returns [out, peak, prevAbs, prevV, still, freeze, hyst, oneEuro].
fn axis(v, dt, st, k) {
    let a = absv(v);
    let peak = st[0];
    let lastSign = st[5];

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
    let hyst = st[4];
    let fall = clampv(fast + (slow - fast) * hyst, 0.0, 1.0);

    // 3) peak tracking
    if a > peak { peak = a; }
    else { peak = peak * exp(-dt / k[4]); }

    // 4) drift hold: long stillness zeroes the axis and its history. Stillness is
    //    judged on the low-passed vector speed (k[14]) so a single noisy sample
    //    cannot keep resetting the timer.
    let still = st[2];
    if k[5] > 0.0 {
        if k[14] < k[6] { still = still + dt; }
        else { still = 0.0; }
        if still >= k[5] {
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        }
    }

    // 5) drop-slope suppression: curve 1 = linear, curve 2 = the original
    //    quadratic. Only these two are implemented (no pow() in the host).
    let raw = 0.0;
    if peak > k[0] {
        let d = clampv(drop * k[7], 0.0, 1.0);
        raw = d;
        if k[13] > 1.5 { raw = d * d; }
    }

    // 6) Both change detectors, enabled together, full strength - which is how
    //    it was always used and never caused trouble. Neither is a "high speed"
    //    detector: what actually gets suppressed is the settling right after a
    //    flick, and there the angular speed is already low.
    // Compare against the last non-zero sample: a crossing that lands exactly
    // on zero is no longer swallowed, which removes the one-sided trigger
    // asymmetry on repeated strokes.
    let event = false;
    if k[8] > 0.0 && peak > k[0] && absv(v) > 0.000001 {
        if absv(lastSign) > 0.000001 && ((v < 0.0) != (lastSign < 0.0)) {
            if absv(v - lastSign) / (dt * 1000.0) > k[8] { raw = 1.0; hyst = 1.0; event = true; }
        }
        lastSign = v;
    }
    // (duplicate one-liner removed: the same override is applied below)

    // 7) vector-reversal override (flag prepared by the caller)
    if k[10] > 0.5 {
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
    let frz = st[3];
    if k[9] > 0.0 {
        if event && hyst > k[11] { frz = k[9]; }
        if frz > 0.0 {
            if hyst > k[11] { frz = k[9]; }
            out = out * k[12];
            frz = frz - dt;
            if frz < 0.0 { frz = 0.0; }
        }
    }

    return [out, peak, a, still, frz, hyst, lastSign];
}

fn process(sample, params) {
    if !this.contains("ok") {
        this.ok = true;
        this.S = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        this.G = [0.0, 0.0, 0.0];
        this.D = [0.0, 0.0];
        this.gOk = false;

        let fall = clampv(pms(params, "fall_ms", "FAAL_HYST_FALL_TIME", 65.0) / 1000.0, 0.0001, 1.0);
        let ff = pms(params, "fall_fast_ms", "FAAL_HYST_FALL_FAST_TIME", 0.0);
        let fastFall = fall * 0.446;
        if ff > 0.0 { fastFall = clampv(ff / 1000.0, 0.0001, fall); }

        // 0 peak_min | 1 rise | 2 fastFall | 3 fall | 4 peak_decay
        // 5 drift_hold | 6 drift_thr | 7 drop_sens | 8 sign_thr | 9 freeze
        // 10 force | 11 suppress_thr | 12 freeze_mul | 13 drop_curve
        this.K = [
            pf(params, "peak_min", "", 2.2),
            clampv(pms(params, "rise_ms", "FAAL_HYST_RISE_TIME", 2.9) / 1000.0, 0.0001, 1.0),
            fastFall,
            fall,
            clampv(pms(params, "peak_decay_ms", "FAAL_PEAK_DECAY_TIME", 65.0) / 1000.0, 0.0001, 1.0),
            clampv(pms(params, "drift_hold_ms", "FAAL_DRIFT_HOLD_TIME_SEC", 200.0) / 1000.0, 0.0, 10.0),
            clampv(pf(params, "drift_thr", "", 0.15), 0.0, 100.0),
            pf(params, "drop_sens", "FAAL_DROP_SENSITIVITY", 2.4),
            pf(params, "sign_thr", "FAAL_SIGN_CHANGE_THR", 0.5),
            clampv(pms(params, "freeze_ms", "FAAL_FREEZE_DURATION_SEC", 24.0) / 1000.0, 0.0, 1.0),
            0.0,
            pf(params, "suppress_thr", "", 0.45),
            clampv(pf(params, "freeze_mul", "", 0.1), 0.0, 1.0),
            pf(params, "drop_curve", "", 2.0),
            0.0,
        ];

        this.smoothTime = clampv(pms(params, "smooth_time_ms", "FAAL_SMOOTH_TIME_SEC", 7.5) / 1000.0, 0.001, 1.0);
        this.stillLpMs = clampv(pms(params, "still_lp_ms", "", 60.0) / 1000.0, 0.001, 1.0);
        this.stillLp = 0.0;
        this.stillLpMs = clampv(pms(params, "still_lp_ms", "", 60.0) / 1000.0, 0.001, 1.0);
        this.stillLp = 0.0;
        this.revOn = pf(params, "rev_on", "FAAL_ENABLE_VECTOR_REVERSAL", 1.0);
        this.revMag = clampv(pf(params, "rev_mag", "FAAL_DIR_CHANGE_MAG_RATIO", 0.15), 0.0, 1.0);
        this.revCos = 0.36235775;   // cos(1.2 rad), the tuned JSM reversal angle

        // Internal 1 ms clock, same scheme as faal_ref.rhai: the host's tick rate
        // must not decide how the filter advances.
        this.acc = 0.0;
        this.report = clampv(pms(params, "report_ms", "", 0.0) / 1000.0, 0.0, 0.05);
        this.period = 0.0;
        // Worst case per sample decides whether the whole chain fits the host's
        // operation budget. Two steps keeps it small; raise it only if the host
        // has budget to spare.
        this.maxSteps = pf(params, "max_steps", "", 1.0);
        if this.maxSteps < 1.0 { this.maxSteps = 1.0; }
        if this.maxSteps > 8.0 { this.maxSteps = 8.0; }
        this.lx = 1e9;
        this.ly = 1e9;
        this.lz = 1e9;
        this.lout = [0.0, 0.0, 0.0];
    }

    let ix = sample.x;
    let iy = sample.y;
    let iz = sample.z;

    // Repeated reports carry no time of their own: hold instead of re-filtering,
    // or the derivative terms read zero on most ticks and spike on the one that
    // changed. Judged by value, not by dt, because an event-timestamp host can
    // hand a genuinely new reading a zero dt.
    if ix == this.lx && iy == this.ly && iz == this.lz {
        return this.lout;
    }

    let dt = sample.dt;
    if dt < 0.0 { dt = 0.0; }
    if dt > 0.05 { dt = 0.05; }

    // A pinned interval wins; otherwise estimate it from the gaps between new
    // readings, so a jittery host clock still advances by the true average period.
    if this.report > 0.0 {
        dt = this.report;
    } else if dt > 0.0 {
        if this.period <= 0.0 { this.period = dt; }
        else if dt > this.period * 3.0 || dt < this.period * 0.34 {
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

    // ---- gravity, low-passed so the basis does not jitter ----
    if sample.accel_fresh {
        let kg = 1.0 - exp(-dt / 0.15);
        let G = this.G;
        G[0] = G[0] + kg * (sample.accel_x - G[0]);
        G[1] = G[1] + kg * (sample.accel_y - G[1]);
        G[2] = G[2] + kg * (sample.accel_z - G[2]);
        this.G = G;
        let gl = sqrt(G[0] * G[0] + G[1] * G[1] + G[2] * G[2]);
        if gl > 1e-6 { this.gOk = true; }
    }

    // ---- player-space pair ----
    let yaw = ix;
    let pitch = iy;
    let roll = iz;
    let ux = 0.0; let uy = 0.0; let uz = 1.0;
    let fx = 1.0; let fy = 0.0; let fz = 0.0;
    let rx = 0.0; let ry = 1.0; let rz = 0.0;
    let usedSpace = false;

    if this.gOk {
        let G = this.G;
        let gl = sqrt(G[0] * G[0] + G[1] * G[1] + G[2] * G[2]);
        if gl > 1e-6 {
            // accelerometers read roughly (0, 0, -1) at rest, so "up" is -accel
            ux = -G[0] / gl;
            uy = -G[1] / gl;
            uz = -G[2] / gl;

            // forward = device X with its vertical part removed
            let ax2 = 1.0 - ux * ux;
            let ay2 = -ux * uy;
            let az2 = -ux * uz;
            let fl = sqrt(ax2 * ax2 + ay2 * ay2 + az2 * az2);
            if fl > 1e-6 {
                fx = ax2 / fl;
                fy = ay2 / fl;
                fz = az2 / fl;

                // right = up x forward
                rx = uy * fz - uz * fy;
                ry = uz * fx - ux * fz;
                rz = ux * fy - uy * fx;
                let rl = sqrt(rx * rx + ry * ry + rz * rz);
                if rl > 1e-6 {
                    rx = rx / rl;
                    ry = ry / rl;
                    rz = rz / rl;

                    // horizontal intent: the gravity-aligned component, capped by
                    // the whole (up, forward) planar magnitude so a tilted device'
                    // roll share still contributes - exactly JSM's
                    //   yaw = sign(w) * min(|w| * 2, sqrt(w^2 + f^2))
                    let wv = ix * ux + iy * uy + iz * uz;
                    let hv = ix * fx + iy * fy + iz * fz;
                    let planar = sqrt(wv * wv + hv * hv);
                    let sgn = 1.0;
                    if wv < 0.0 { sgn = -1.0; }
                    let aw = absv(wv) * 2.0;
                    yaw = sgn * (if aw < planar { aw } else { planar });
                    pitch = ix * rx + iy * ry + iz * rz;
                    roll = hv;
                    usedSpace = true;
                }
            }
        }
    }

    // ---- FAAL on the player-space pair, in whole 1 ms steps ----
    let speed = sqrt(yaw * yaw + pitch * pitch);

    let fyaw = this.lout[0];
    let fpitch = this.lout[1];
    let froll = this.lout[2];
    let steps = 0;

    while this.acc >= 0.001 && steps < this.maxSteps {
        // Spend it in one step when the remaining steps could not cover it: the
        // per-sample operation budget is finite and an overrun bypasses the whole
        // filter. This keeps the cost at maxSteps axis-pairs per sample.
        let big = this.acc > (this.maxSteps - steps + 1.0) * 0.001;
        let step = if big { this.acc } else { 0.001 };
        this.acc = if big { 0.0 } else { this.acc - 0.001 };
        steps = steps + 1;

        let K = this.K;
        // stillness is judged on a 60 ms low-pass of the vector speed
        let la = 1.0 - exp(-step / this.stillLpMs);
        this.stillLp = this.stillLp + la * (speed - this.stillLp);
        K[14] = this.stillLp;
        K[10] = 0.0;

        let alpha = 1.0 - exp(-step / this.smoothTime);
        if this.revOn > 0.5 {
            let pl = sqrt(this.D[0] * this.D[0] + this.D[1] * this.D[1]);
            if pl > 0.000001 && speed > 0.000001 {
                let c = clampv((this.D[0] * yaw + this.D[1] * pitch) / (pl * speed), -1.0, 1.0);
                if c < this.revCos {
                    let s2 = 1.0 - c * c;
                    if s2 > 0.0 && sqrt(s2) > this.revMag { K[10] = 1.0; }
                }
            }
        }
        this.D[0] = this.D[0] + alpha * (yaw - this.D[0]);
        this.D[1] = this.D[1] + alpha * (pitch - this.D[1]);
        this.K = K;

        let S = this.S;
        let sy = axis(yaw, step, [S[0], S[1], S[2], S[3], S[4], S[5]], K);
        let sp = axis(pitch, step, [S[6], S[7], S[8], S[9], S[10], S[11]], K);

        let sz0 = this.lout[2];
        if !usedSpace {
            // No usable gravity. Filter the third device axis too, so the
            // horizontal one cannot slip through untouched.
            let sz = axis(iz, step, [S[12], S[13], S[14], S[15], S[16], S[17]], K);
            this.S = [sy[1], sy[2], sy[3], sy[4], sy[5], sy[6],
                      sp[1], sp[2], sp[3], sp[4], sp[5], sp[6],
                      sz[1], sz[2], sz[3], sz[4], sz[5], sz[6]];
            sz0 = sz[0];
        } else {
            this.S = [sy[1], sy[2], sy[3], sy[4], sy[5], sy[6],
                      sp[1], sp[2], sp[3], sp[4], sp[5], sp[6],
                      S[12], S[13], S[14], S[15], S[16], S[17]];
        }

        fyaw = sy[0];
        fpitch = sp[0];
        froll = sz0;
        if fyaw != fyaw || fpitch != fpitch {
            this.ok = false;
            return [sample.x, sample.y, sample.z];
        }
    }

    this.lx = ix;
    this.ly = iy;
    this.lz = iz;

    if !usedSpace {
        this.lout = [fyaw, fpitch, froll];
        return this.lout;
    }

    // ---- PLAYER mode: scale the original vector, do not rewrite its direction ----
    // JSM scales both turn axes by the yaw ratio and the pitch axis by the pitch
    // ratio. Here the turn pair is (up, forward), which is why the forward share
    // is scaled too: that is the roll contribution the horizontal intent borrowed.
    let kYaw = 1.0;
    let kPitch = 1.0;
    if absv(yaw) > 0.001 { kYaw = clampv(fyaw / yaw, 0.0, 1.0); }
    if absv(pitch) > 0.001 { kPitch = clampv(fpitch / pitch, 0.0, 1.0); }

    let wv = ix * ux + iy * uy + iz * uz;
    let hv = ix * fx + iy * fy + iz * fz;
    let pv2 = ix * rx + iy * ry + iz * rz;
    this.lout = [
        (wv * kYaw) * ux + (pv2 * kPitch) * rx + (hv * kYaw) * fx,
        (wv * kYaw) * uy + (pv2 * kPitch) * ry + (hv * kYaw) * fy,
        (wv * kYaw) * uz + (pv2 * kPitch) * rz + (hv * kYaw) * fz
    ];
    return this.lout;
}
