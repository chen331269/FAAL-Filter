#include "faal_filter.h"
#include <math.h>
#include <stdio.h>

static FaalParams g_params;

static AxisState s_faal_stateX;
static AxisState s_faal_stateY;
static AxisState s_faal_stateZ;
static bool s_faal_firstFrame = true;

static float s_smoothDirX = 0.0f;
static float s_smoothDirY = 0.0f;
static float s_smoothDirZ = 0.0f;

// ---------------------------------------------------------------------------
//  Internal constants
// ---------------------------------------------------------------------------
static const float kDtMin = 1e-3f;            // 1 ms
static const float kDtMax = 0.05f;            // 50 ms
static const float kDropSpeedRefHz = 1000.0f; // 1000Hz reference for dropSpeed/signChange

static inline float faal_abs(float v)
{
	return (v < 0.0f) ? -v : v;
}

static inline float faal_clamp(float v, float lo, float hi)
{
	if (v < lo)
		return lo;
	if (v > hi)
		return hi;
	return v;
}

// ---------------------------------------------------------------------------
//  One Euro Filter (passthrough derivative, alpha_d = 1)
// ---------------------------------------------------------------------------
static float oneEuroFilter(float x, float dt, float fc_min, float beta,
  float& x_prev, float& dx_prev)
{
	float dx = (x - x_prev) / dt;
	dx_prev = dx;

	float fc = fc_min + beta * faal_abs(dx_prev);
	float tau = 1.0f / (2.0f * 3.14159265f * fc + 0.00001f);
	float alpha = 1.0f / (1.0f + tau / dt);

	float x_out = alpha * x + (1.0f - alpha) * x_prev;
	x_prev = x_out;
	return x_out;
}

// ---------------------------------------------------------------------------
//  Per-axis processing
// ---------------------------------------------------------------------------
// Same filter, but the cutoff is decided by the caller: the tier split needs the
// speed of the whole vector, so the per-axis function must not derive its own.
static float oneEuroWithCutoff(float x, float dt, float fc, float& x_prev)
{
	float tau = 1.0f / (2.0f * 3.14159265f * fc + 0.00001f);
	float alpha = 1.0f / (1.0f + tau / dt);
	float x_out = alpha * x + (1.0f - alpha) * x_prev;
	x_prev = x_out;
	return x_out;
}

// 0 below e0, 1 above e1, smooth in between.
static float smoothstepf(float e0, float e1, float x)
{
	if (e1 <= e0)
		return (x >= e1) ? 1.0f : 0.0f;
	float t = (x - e0) / (e1 - e0);
	t = faal_clamp(t, 0.0f, 1.0f);
	return t * t * (3.0f - 2.0f * t);
}
// Tier weight for the soft-tiered split: 0 = fully smoothed, 1 = fully direct.
static float smoothTierWeight(float speed)
{
	float top = g_params.smoothThreshold;
	if (top <= 0.0f)
		return 1.0f;
	float bottom = top * 0.5f;
	if (speed <= bottom)
		return 0.0f;
	if (speed >= top)
		return 1.0f;
	return (speed - bottom) / (top - bottom);
}
static float doFilter(AxisState& st, float v, float dt, bool forceSuppress,
  float tierWeight, float vectorSpeed)
{
	float absV = faal_abs(v);

	// ---- 1) dropSpeed (1000 Hz normalized) ------------------------------
	float dropSpeed = 0.0f;
	if (st.peakValue > g_params.peakMinTrigger)
	{
		float deltaAbsV = st.prevAbsV - absV;
		if (deltaAbsV > 0.0f)
		{
			dropSpeed = deltaAbsV / (dt * kDropSpeedRefHz) /
			  (st.peakValue + 0.001f);
		}
	}

	// ---- 2) Frame-rate independent hysteresis coefficients --------------
	float riseFactor = 1.0f - expf(-dt / g_params.hystRiseTime);
	float fastFallFactor = 1.0f - expf(-dt / g_params.hystFallFastTime);
	float slowFallFactor = 1.0f - expf(-dt / g_params.hystFallTime);

	// ---- 3) Adaptive fall factor ----------------------------------------
	// Use the PREVIOUS frame's suppressHyst as the adaptive signal.
	// Strong suppression (flick just ended) → slow fall (sticky).
	// Weak suppression (normal tracking)    → fast fall (responsive).
	// No thresholds. State drives itself.
	float fallFactor = fastFallFactor +
	  (slowFallFactor - fastFallFactor) * st.suppressHyst;
	fallFactor = faal_clamp(fallFactor, 0.0f, 1.0f);

	// ---- 4) Peak tracking (independent peakDecayTime) -------------------
	st.prevPeakValue = st.peakValue;
	if (absV > st.peakValue)
		st.peakValue = absV;
	else
		st.peakValue *= expf(-dt / g_params.peakDecayTime);

	// ---- 5) Drift hold --------------------------------------------------
	if (g_params.enableDriftHold && g_params.driftHoldTimeSec > 0.0f)
	{
		if (absV < g_params.driftThreshold)
			st.stillTimeSec += dt;
		else
			st.stillTimeSec = 0.0f;

		if (st.stillTimeSec >= g_params.driftHoldTimeSec)
		{
			st.prevOut = st.peakValue = st.prevPeakValue = st.prevAbsV = st.prevV = 0.0f;
			st.suppressHyst = 0.0f;
			st.freezeRemainingSec = 0.0f;
			st.stillTimeSec = 0.0f;
			st.oneEuroPrev = 0.0f;
			st.oneEuroDeriv = 0.0f;
			return 0.0f;
		}
	}

	// ---- 6) Raw suppression ---------------------------------------------
	float rawSuppress = 0.0f;
	if (st.peakValue > g_params.peakMinTrigger)
	{
		float raw = faal_clamp(dropSpeed * g_params.dropSensitivity, 0.0f, 1.0f);
		rawSuppress = powf(raw, g_params.dropCurve);
	}

	// ---- 7) Single-frame zero-crossing override -------------------------
	if (g_params.signChangeThr > 0.0f && st.peakValue > g_params.peakMinTrigger)
	{
		bool bothNonZero = (st.prevV != 0.0f) && (v != 0.0f);
		bool signedFlip = bothNonZero && ((st.prevV < 0.0f) != (v < 0.0f));
		float refDeltaV = faal_abs(v - st.prevV) / (dt * kDropSpeedRefHz);
		bool magnitudeJump = refDeltaV > g_params.signChangeThr;
		if (signedFlip && magnitudeJump)
		{
			rawSuppress = 1.0f;
			st.suppressHyst = 1.0f;
		}
	}

	// ---- 8) External override (vector reversal) -------------------------
	if (forceSuppress)
	{
		rawSuppress = 1.0f;
		st.suppressHyst = 1.0f;
	}

	// ---- 8b) Hand-off gate ----------------------------------------------
	// Below the smoothing threshold the signal cannot separate intent from
	// tremor, so suppression stays out of it and the low-pass does the work.
	if (g_params.smoothThreshold > 0.0f)
	{
		float gate = smoothstepf(g_params.smoothThreshold,
		  2.0f * g_params.smoothThreshold, vectorSpeed);
		rawSuppress *= gate;
	}

	// ---- 9) Hysteresis (self-adaptive fall) -----------------------------
	if (rawSuppress > st.suppressHyst)
	{
		st.suppressHyst = st.suppressHyst * (1.0f - riseFactor) +
		  rawSuppress * riseFactor;
	}
	else
	{
		st.suppressHyst = st.suppressHyst * (1.0f - fallFactor) +
		  rawSuppress * fallFactor;
	}
	st.suppressHyst = faal_clamp(st.suppressHyst, 0.0f, 1.0f);

	st.prevAbsV = absV;

	// ---- 10) Tiered One Euro smoothing ----------------------------------
	// The input is split by magnitude rather than the output blended: whatever goes
	// into the filter is eventually consumed, so displacement is preserved and a big
	// movement never loads the filter's state.
	float directPart = v * tierWeight;
	float smoothed = oneEuroWithCutoff(v * (1.0f - tierWeight), dt,
	  g_params.oneEuroFcMin + g_params.oneEuroBeta * vectorSpeed,
	  st.oneEuroPrev);
	float out = directPart + smoothed;

	// ---- 11) Apply suppression ------------------------------------------
	out *= (1.0f - st.suppressHyst);

	// ---- 12) Freeze window ----------------------------------------------
	if (g_params.freezeDurationSec > 0.0f)
	{
		if (st.suppressHyst > g_params.suppressThr)
			st.freezeRemainingSec = g_params.freezeDurationSec;
		if (st.freezeRemainingSec > 0.0f)
		{
			out *= g_params.freezeOutMul;
			st.freezeRemainingSec -= dt;
			if (st.freezeRemainingSec < 0.0f)
				st.freezeRemainingSec = 0.0f;
		}
	}

	st.prevOut = out;
	st.prevV = v;
	return out;
}

// ---------------------------------------------------------------------------
//  Vector reversal detection
// ---------------------------------------------------------------------------
static bool checkVectorReversal3D(float curX, float curY, float curZ,
  float prevX, float prevY, float prevZ)
{
	if (g_params.enableVectorReversal <= 0.5f)
		return false;

	float prevLen2 = prevX * prevX + prevY * prevY + prevZ * prevZ;
	float curLen2 = curX * curX + curY * curY + curZ * curZ;
	if (prevLen2 < 1e-12f || curLen2 < 1e-12f)
		return false;

	float prevLen = sqrtf(prevLen2);
	float curLen = sqrtf(curLen2);
	float dot = prevX * curX + prevY * curY + prevZ * curZ;
	float cosTheta = faal_clamp(dot / (prevLen * curLen), -1.0f, 1.0f);

	if (cosTheta >= cosf(g_params.dirChangeAngleRad))
		return false;

	float sinTheta = sqrtf(faal_clamp(1.0f - cosTheta * cosTheta, 0.0f, 1.0f));
	return sinTheta > g_params.dirChangeMagRatio;
}

// ---------------------------------------------------------------------------
//  Two-axis API (independent, does not touch Z state)
// ---------------------------------------------------------------------------
void faal_filter(float& outX, float& outY, float dt)
{
	dt = faal_clamp(dt, kDtMin, kDtMax);

	if (s_faal_firstFrame)
	{
		s_faal_stateX.init();
		s_faal_stateY.init();
		s_faal_firstFrame = false;
		s_smoothDirX = s_smoothDirY = 0.0f;
	}

	float alpha = 1.0f - expf(-dt / g_params.smoothTimeSec);
	bool forceSuppress = false;
	if (g_params.enableVectorReversal > 0.5f)
	{
		float prevLen2 = s_smoothDirX * s_smoothDirX + s_smoothDirY * s_smoothDirY;
		float curLen2 = outX * outX + outY * outY;
		if (prevLen2 > 1e-12f && curLen2 > 1e-12f)
		{
			float prevLen = sqrtf(prevLen2);
			float curLen = sqrtf(curLen2);
			float dot = s_smoothDirX * outX + s_smoothDirY * outY;
			float cosTheta = faal_clamp(dot / (prevLen * curLen), -1.0f, 1.0f);
			if (cosTheta < cosf(g_params.dirChangeAngleRad))
			{
				float sinTheta = sqrtf(faal_clamp(1.0f - cosTheta * cosTheta, 0.0f, 1.0f));
				forceSuppress = sinTheta > g_params.dirChangeMagRatio;
			}
		}
	}

	// Compare first, then absorb (updating first under-reads the turn).
	s_smoothDirX += alpha * (outX - s_smoothDirX);
	s_smoothDirY += alpha * (outY - s_smoothDirY);

	float speed2 = sqrtf(outX * outX + outY * outY);
	float w2 = smoothTierWeight(speed2);
	outX = doFilter(s_faal_stateX, outX, dt, forceSuppress, w2, speed2);
	outY = doFilter(s_faal_stateY, outY, dt, forceSuppress, w2, speed2);
}

// ---------------------------------------------------------------------------
//  Three-axis API
// ---------------------------------------------------------------------------
void faal_filter(float& outX, float& outY, float& outZ, float dt)
{
	dt = faal_clamp(dt, kDtMin, kDtMax);

	if (s_faal_firstFrame)
	{
		s_faal_stateX.init();
		s_faal_stateY.init();
		s_faal_stateZ.init();
		s_faal_firstFrame = false;
		s_smoothDirX = s_smoothDirY = s_smoothDirZ = 0.0f;
	}

	float alpha = 1.0f - expf(-dt / g_params.smoothTimeSec);
	bool forceSuppress = checkVectorReversal3D(
	  outX, outY, outZ,
	  s_smoothDirX, s_smoothDirY, s_smoothDirZ);

	// Compare first, then absorb (see the two-axis entry point).
	s_smoothDirX += alpha * (outX - s_smoothDirX);
	s_smoothDirY += alpha * (outY - s_smoothDirY);
	s_smoothDirZ += alpha * (outZ - s_smoothDirZ);

	float speed3 = sqrtf(outX * outX + outY * outY + outZ * outZ);
	float w3 = smoothTierWeight(speed3);
	outX = doFilter(s_faal_stateX, outX, dt, forceSuppress, w3, speed3);
	outY = doFilter(s_faal_stateY, outY, dt, forceSuppress, w3, speed3);
	outZ = doFilter(s_faal_stateZ, outZ, dt, forceSuppress, w3, speed3);
}

void ResetFaalFilter()
{
	s_faal_firstFrame = true;
	s_faal_stateX.init();
	s_faal_stateY.init();
	s_faal_stateZ.init();
	s_smoothDirX = s_smoothDirY = s_smoothDirZ = 0.0f;
}

const FaalParams& GetFaalParams()
{
	return g_params;
}

void SetFaalParams(const FaalParams& p)
{
	g_params = p;
	// DERIVED: fast fall = 0.446 × fall time (from original 0.029 / 0.065)
	g_params.hystFallFastTime = g_params.hystFallTime * 0.446f;
}