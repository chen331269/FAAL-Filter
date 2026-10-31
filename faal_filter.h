#pragma once

// ============================================================================
//  FAAL filter — public API + parameters + per-axis state
//  Frame-rate independent version.
//
//  Key guarantees:
//   * Hysteresis / peak decay / smoothing use exp(-dt/tau)             => FRI
//   * dropSpeed and sign-change deltas are normalized to a 1000 Hz
//     reference so the same physical motion produces the same value
//     at any actual tick rate
//   * Freeze window and drift hold accumulate TIME (float seconds)
//   * Vector reversal uses pure angular comparison
//   * Two-axis API is fully independent: does not touch Z-axis state
//   * One Euro exposes only fc_min and beta; derivative channel is a
//     passthrough (alpha_d = 1)
//
//  Hysteresis (adaptive fall, no thresholds / no magic numbers):
//   * RISE_TIME     : ramp-up time (s)                 — user
//   * FALL_TIME     : ramp-down time at full suppress  — user
//   * PEAK_DECAY    : peak tracking decay time (s)     — user (independent)
//   * fallFastTime  = FALL_TIME × 0.446                — derived
//   Per-frame adaptive fall:
//     fall = fastFall + (slowFall - fastFall) × suppressHyst
//   Strong suppression (flick just ended) → slow (sticky)
//   Weak suppression (normal tracking)    → fast (responsive)
// ============================================================================

struct FaalParams
{
	// ---- Peak tracking --------------------------------------------------
	// Independent user parameter; controls how fast the peak tracker decays.
	float peakDecayTime = 0.065f;
	float peakMinTrigger = 2.0f;

	// ---- Drop-slope suppression -----------------------------------------
	float dropSensitivity = 2.5f;
	float dropCurve = 2.0f;

	// ---- Suppress hysteresis (adaptive fall) ----------------------------
	// USER: hystRiseTime, hystFallTime, peakDecayTime
	// DERIVED: hystFallFastTime = hystFallTime × 0.446
	float hystRiseTime = 0.0029f;
	float hystFallTime = 0.065f;
	float hystFallFastTime = 0.029f; // DERIVED

	// ---- Optional freeze window -----------------------------------------
	float suppressThr = 0.5f;
	float freezeDurationSec = 0.0f;
	float freezeOutMul = 0.2f;

	// ---- Sign-change trigger --------------------------------------------
	float signChangeThr = 1.5f;

	// ---- Drift hold ------------------------------------------------------
	bool enableDriftHold = true;
	float driftThreshold = 0.15f;
	float driftHoldTimeSec = 0.2f;

	// ---- One Euro Filter -------------------------------------------------
	// Only fc_min and beta. Derivative passthrough (alpha_d = 1).
	float oneEuroFcMin = 2.0f;
	float oneEuroBeta = 0.08f;

	// ---- Vector reversal detection --------------------------------------
	float enableVectorReversal = 0.0f;
	float dirChangeMagRatio = 0.3f;
	float dirChangeAngleRad = 1.5f;

	// ---- Smoothing time for direction EWMA ------------------------------
	float smoothTimeSec = 0.020f;
};

struct AxisState
{
	float smoothedSpeedDiff;
	float prevShapedNorm;
	float ringData[32];
	int ringCount = 0;
	int ringIdx = 0;
	float ringAvg = 0.0f;

	float peakValue = 0.0f;
	float prevPeakValue = 0.0f;
	float prevOut = 0.0f;
	float prevAbsV = 0.0f;
	float prevV = 0.0f;

	// Accumulated TIME (seconds) — frame-rate independent.
	float stillTimeSec = 0.0f;
	float freezeRemainingSec = 0.0f;

	float suppressHyst = 0.0f;

	float oneEuroPrev = 0.0f;
	float oneEuroDeriv = 0.0f;

	void init()
	{
		smoothedSpeedDiff = 0.0f;
		prevShapedNorm = 0.0f;
		ringCount = ringIdx = 0;
		ringAvg = peakValue = prevPeakValue = prevOut = prevAbsV = prevV = 0.0f;
		stillTimeSec = freezeRemainingSec = 0.0f;
		suppressHyst = 0.0f;
		oneEuroPrev = 0.0f;
		oneEuroDeriv = 0.0f;
		for (int i = 0; i < 32; ++i)
			ringData[i] = 0.0f;
	}
};

void faal_filter(float& outX, float& outY, float dt);
void faal_filter(float& outX, float& outY, float& outZ, float dt);
void ResetFaalFilter();
const FaalParams& GetFaalParams();
void SetFaalParams(const FaalParams& p);