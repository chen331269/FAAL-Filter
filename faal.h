/*
 * faal.h - FAAL anti-shake filter, C99 ABI.
 *
 * One header, one source file, no dependencies. Every language can either
 * compile faal.c into itself or call the prebuilt library through FFI.
 *
 * Threading: a context must not be used from two threads at once. Parameters
 * may be changed from the same thread between calls.
 */
#ifndef FAAL_H
#define FAAL_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(FAAL_BUILD_DLL)
#  define FAAL_API __declspec(dllexport)
#elif defined(FAAL_USE_DLL)
#  define FAAL_API __declspec(dllimport)
#else
#  define FAAL_API
#endif

/* ABI version. Bump only when an existing field or function changes meaning. */
#define FAAL_ABI_VERSION 1

/*
 * Parameters. The first field is always `size`: fill it with sizeof(FaalParams).
 * Fields appended later are only read when `size` covers them, so old callers
 * keep working against a newer library.
 */
typedef struct FaalParams
{
	unsigned int size;

	/* peak tracking */
	float peakDecayTime;    /* s, peak decay when the input stops rising */
	float peakMinTrigger;   /* deg/s, below this the peak tracker is ignored */

	/* drop-slope suppression */
	float dropSensitivity;  /* gain applied to the drop rate */
	float dropCurve;        /* exponent applied to the suppressed result */

	/* hysteresis */
	float hystRiseTime;     /* s, how fast suppression engages */
	float hystFallTime;     /* s, how fast it releases */

	/* freeze window (0 duration disables it) */
	float suppressThr;      /* 0..1, above this the freeze window re-arms */
	float freezeDurationSec;
	float freezeOutMul;     /* output multiplier while frozen */

	/* sign-change override (0 disables it) */
	float signChangeThr;

	/* drift hold */
	int   enableDriftHold;
	float driftThreshold;   /* deg/s */
	float driftHoldTimeSec;

	/* smoothing */
	float oneEuroFcMin;     /* Hz, cutoff floor */
	float oneEuroBeta;      /* Hz per deg/s */
	float smoothThreshold;  /* deg/s, above this nothing is smoothed. 0 = no smoothing */

	/* vector reversal */
	int   enableVectorReversal;
	float dirChangeMagRatio;
	float dirChangeAngleRad;
	float smoothTimeSec;    /* s, memory of the reversal reference */
} FaalParams;

typedef struct Faal Faal;

/* Convenience path: the library allocates. Returns NULL on allocation failure. */
FAAL_API Faal *faal_new(void);
FAAL_API void  faal_free(Faal *ctx);

/* Embedded path: caller-provided storage, no allocation at all. */
FAAL_API unsigned int faal_size(void);
FAAL_API void faal_init(Faal *ctx, const FaalParams *params);

FAAL_API void faal_defaults(FaalParams *out);
FAAL_API void faal_set_params(Faal *ctx, const FaalParams *params);

/* Filters in place. dim is 2 or 3. dt is in seconds. */
FAAL_API void faal_process(Faal *ctx, float *v, int dim, float dt);

FAAL_API void faal_reset(Faal *ctx);

#ifdef __cplusplus
}
#endif

/* ------------------------------ implementation ------------------------------ */
#ifdef FAAL_IMPLEMENTATION

#ifdef __cplusplus
extern "C" {
#endif

/*
 * faal.c - FAAL anti-shake filter, C99 implementation.
 *
 * Layout and maths mirror the C++ implementation used by JoyShockMapper, so the
 * two produce the same numbers for the same input.
 */
/* implementation included from the same header */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define FAAL_PI 3.14159265358979323846

typedef struct Axis
{
	float peakValue;
	float prevAbsV;
	float prevV;
	float stillTimeSec;
	float freezeRemainingSec;
	float suppressHyst;
	float oneEuroPrev;
} Axis;

struct Faal
{
	FaalParams params;
	float hystFallFastTime; /* derived: hystFallTime * 0.446 */
	Axis ax;
	Axis ay;
	Axis az;
	int first;
	int mode; /* 0 unused, 2 two-axis, 3 three-axis */
	float dirX, dirY, dirZ;
};

static float clampf(float v, float lo, float hi)
{
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static void axis_reset(Axis *a)
{
	memset(a, 0, sizeof(*a));
}

static void ctx_reset(Faal *ctx)
{
	axis_reset(&ctx->ax);
	axis_reset(&ctx->ay);
	axis_reset(&ctx->az);
	ctx->first = 1;
	ctx->dirX = ctx->dirY = ctx->dirZ = 0.0f;
}

static float one_euro(float x, float dt, float fc, float *prev)
{
	float tau = 1.0f / (2.0f * (float)FAAL_PI * fc + 0.00001f);
	float alpha = 1.0f / (1.0f + tau / dt);
	float out = alpha * x + (1.0f - alpha) * *prev;
	*prev = out;
	return out;
}

/* Soft-tiered split: 0 = fully smoothed, 1 = fully direct. */
static float tier_weight(const FaalParams *p, float speed)
{
	float top = p->smoothThreshold;
	float bottom;

	if (top <= 0.0f)
		return 1.0f; /* threshold 0: no smoothing at all */

	bottom = top * 0.5f;
	if (speed <= bottom) return 0.0f;
	if (speed >= top) return 1.0f;
	return (speed - bottom) / (top - bottom);
}

static float do_filter(const FaalParams *p, Axis *st, float v, float dt,
                       int forceSuppress, float tierWeight, float vectorSpeed)
{
	float absV = fabsf(v);
	float dropSpeed = 0.0f;
	float riseFactor, fastFallFactor, slowFallFactor, fallFactor;
	float rawSuppress = 0.0f;
	float out, directPart, smoothed;

	/* 1) drop rate, normalised to 1 kHz and to the current peak */
	if (st->peakValue > p->peakMinTrigger)
	{
		float deltaAbsV = st->prevAbsV - absV;
		if (deltaAbsV > 0.0f)
			dropSpeed = deltaAbsV / (dt * 1000.0f) / (st->peakValue + 0.001f);
	}

	/* 2) frame-rate independent hysteresis coefficients */
	riseFactor = 1.0f - expf(-dt / p->hystRiseTime);
	fastFallFactor = 1.0f - expf(-dt / (p->hystFallTime * 0.446f));
	slowFallFactor = 1.0f - expf(-dt / p->hystFallTime);
	fallFactor = fastFallFactor + (slowFallFactor - fastFallFactor) * st->suppressHyst;
	fallFactor = clampf(fallFactor, 0.0f, 1.0f);

	/* 3) peak tracking */
	if (absV > st->peakValue)
		st->peakValue = absV;
	else
		st->peakValue *= expf(-dt / p->peakDecayTime);

	/* 4) drift hold */
	if (p->enableDriftHold && p->driftHoldTimeSec > 0.0f)
	{
		if (absV < p->driftThreshold)
			st->stillTimeSec += dt;
		else
			st->stillTimeSec = 0.0f;

		if (st->stillTimeSec >= p->driftHoldTimeSec)
		{
			axis_reset(st);
			return 0.0f;
		}
	}

	/* 5) drop-slope suppression */
	if (st->peakValue > p->peakMinTrigger)
	{
		float raw = clampf(dropSpeed * p->dropSensitivity, 0.0f, 1.0f);
		rawSuppress = powf(raw, p->dropCurve);
	}

	/* 6) sign-change override */
	if (p->signChangeThr > 0.0f && st->peakValue > p->peakMinTrigger)
	{
		int bothNonZero = (st->prevV != 0.0f) && (v != 0.0f);
		int signedFlip = bothNonZero && ((st->prevV < 0.0f) != (v < 0.0f));
		float refDeltaV = fabsf(v - st->prevV) / (dt * 1000.0f);
		if (signedFlip && refDeltaV > p->signChangeThr)
		{
			rawSuppress = 1.0f;
			st->suppressHyst = 1.0f;
		}
	}

	/* 7) vector reversal override */
	if (forceSuppress)
	{
		rawSuppress = 1.0f;
		st->suppressHyst = 1.0f;
	}

	/* 8) hysteresis */
	if (rawSuppress > st->suppressHyst)
		st->suppressHyst = st->suppressHyst * (1.0f - riseFactor) + rawSuppress * riseFactor;
	else
		st->suppressHyst = st->suppressHyst * (1.0f - fallFactor) + rawSuppress * fallFactor;
	st->suppressHyst = clampf(st->suppressHyst, 0.0f, 1.0f);

	st->prevAbsV = absV;

	/* 9) tiered One Euro. The filter is fed the slow share only, and is fed a
	 *    zero share when everything is direct so its state drains. */
	directPart = v * tierWeight;
	smoothed = one_euro(v * (1.0f - tierWeight), dt,
	                    p->oneEuroFcMin + p->oneEuroBeta * vectorSpeed,
	                    &st->oneEuroPrev);
	out = directPart + smoothed;

	/* 10) suppression and freeze window */
	out *= (1.0f - st->suppressHyst);

	if (p->freezeDurationSec > 0.0f)
	{
		if (st->suppressHyst > p->suppressThr)
			st->freezeRemainingSec = p->freezeDurationSec;
		if (st->freezeRemainingSec > 0.0f)
		{
			out *= p->freezeOutMul;
			st->freezeRemainingSec -= dt;
			if (st->freezeRemainingSec < 0.0f)
				st->freezeRemainingSec = 0.0f;
		}
	}

	st->prevV = v;
	return out;
}

static int check_reversal_3d(const FaalParams *p,
                             float cx, float cy, float cz,
                             float px, float py, float pz)
{
	float prevLen2, curLen2, prevLen, curLen, cosTheta, sinTheta;

	if (!p->enableVectorReversal)
		return 0;

	prevLen2 = px * px + py * py + pz * pz;
	curLen2 = cx * cx + cy * cy + cz * cz;
	if (prevLen2 < 1e-12f || curLen2 < 1e-12f)
		return 0;

	prevLen = sqrtf(prevLen2);
	curLen = sqrtf(curLen2);
	cosTheta = clampf((px * cx + py * cy + pz * cz) / (prevLen * curLen), -1.0f, 1.0f);

	if (cosTheta >= cosf(p->dirChangeAngleRad))
		return 0;

	sinTheta = sqrtf(clampf(1.0f - cosTheta * cosTheta, 0.0f, 1.0f));
	return sinTheta > p->dirChangeMagRatio;
}

/* ------------------------------------------------------------------------- */
/* public API                                                                */
/* ------------------------------------------------------------------------- */

unsigned int faal_size(void)
{
	return (unsigned int)sizeof(Faal);
}

Faal *faal_new(void)
{
	Faal *ctx = (Faal *)malloc(sizeof(Faal));
	FaalParams p;

	if (!ctx)
		return NULL;
	memset(ctx, 0, sizeof(*ctx));
	faal_defaults(&p);
	faal_init(ctx, &p);
	return ctx;
}

void faal_free(Faal *ctx)
{
	free(ctx);
}

void faal_defaults(FaalParams *out)
{
	if (!out)
		return;
	memset(out, 0, sizeof(*out));

	out->size = (unsigned int)sizeof(FaalParams);

	out->peakDecayTime = 0.065f;
	out->peakMinTrigger = 2.0f;

	out->dropSensitivity = 2.5f;
	out->dropCurve = 2.0f;

	out->hystRiseTime = 0.0029f;
	out->hystFallTime = 0.065f;

	out->suppressThr = 0.5f;
	out->freezeDurationSec = 0.0f;
	out->freezeOutMul = 0.2f;

	out->signChangeThr = 1.5f;

	out->enableDriftHold = 1;
	out->driftThreshold = 0.15f;
	out->driftHoldTimeSec = 0.2f;

	out->oneEuroFcMin = 2.0f;
	out->oneEuroBeta = 0.5f;
	out->smoothThreshold = 0.0f;

	out->enableVectorReversal = 0;
	out->dirChangeMagRatio = 0.3f;
	out->dirChangeAngleRad = 1.570796f;
	out->smoothTimeSec = 0.1f;
}

void faal_init(Faal *ctx, const FaalParams *params)
{
	if (!ctx)
		return;
	memset(ctx, 0, sizeof(*ctx));
	ctx->first = 1;
	ctx->mode = 0;
	if (params)
		faal_set_params(ctx, params);
	else
	{
		FaalParams d;
		faal_defaults(&d);
		ctx->params = d;
		ctx->hystFallFastTime = d.hystFallTime * 0.446f;
	}
}

void faal_set_params(Faal *ctx, const FaalParams *params)
{
	FaalParams p;

	if (!ctx || !params)
		return;

	p = *params;
	if (p.size == 0)
		p.size = (unsigned int)sizeof(FaalParams);

	/* Only clamp what would otherwise divide by zero or blow up. */
	p.hystRiseTime = clampf(p.hystRiseTime, 1e-4f, 1.0f);
	p.hystFallTime = clampf(p.hystFallTime, 1e-4f, 1.0f);
	p.peakDecayTime = clampf(p.peakDecayTime, 1e-4f, 1.0f);
	p.smoothTimeSec = clampf(p.smoothTimeSec, 1e-3f, 1.0f);
	p.dropCurve = clampf(p.dropCurve, 0.1f, 10.0f);
	p.oneEuroFcMin = clampf(p.oneEuroFcMin, 0.01f, 100.0f);

	ctx->params = p;
	ctx->hystFallFastTime = p.hystFallTime * 0.446f;
}

void faal_reset(Faal *ctx)
{
	if (!ctx)
		return;
	ctx_reset(ctx);
	ctx->mode = 0;
}

void faal_process(Faal *ctx, float *v, int dim, float dt)
{
	const FaalParams *p;
	int forceSuppress = 0;
	float speed, w, alpha;
	float x, y, z = 0.0f;

	if (!ctx || !v || (dim != 2 && dim != 3))
		return;

	/* Switching dimensionality resets, so the two paths never share stale state. */
	if (ctx->mode != 0 && ctx->mode != dim)
	{
		ctx_reset(ctx);
		ctx->first = 1;
	}
	ctx->mode = dim;

	dt = clampf(dt, 1e-3f, 0.05f);

	p = &ctx->params;
	x = v[0];
	y = v[1];
	if (dim == 3)
		z = v[2];

	if (ctx->first)
	{
		ctx_reset(ctx);
		ctx->first = 0;
	}

	alpha = 1.0f - expf(-dt / p->smoothTimeSec);

	/* Reversal is judged against the reference *before* absorbing this sample. */
	if (p->enableVectorReversal)
	{
		if (dim == 2)
		{
			float prevLen2 = ctx->dirX * ctx->dirX + ctx->dirY * ctx->dirY;
			float curLen2 = x * x + y * y;
			if (prevLen2 > 1e-12f && curLen2 > 1e-12f)
			{
				float cosTheta = clampf((ctx->dirX * x + ctx->dirY * y) /
				                        (sqrtf(prevLen2) * sqrtf(curLen2)),
				                        -1.0f, 1.0f);
				if (cosTheta < cosf(p->dirChangeAngleRad))
					forceSuppress = sqrtf(clampf(1.0f - cosTheta * cosTheta, 0.0f, 1.0f)) >
					                p->dirChangeMagRatio;
			}
		}
		else
		{
			forceSuppress = check_reversal_3d(p, x, y, z, ctx->dirX, ctx->dirY, ctx->dirZ);
		}
	}

	ctx->dirX += alpha * (x - ctx->dirX);
	ctx->dirY += alpha * (y - ctx->dirY);
	if (dim == 3)
		ctx->dirZ += alpha * (z - ctx->dirZ);

	if (dim == 2)
		speed = sqrtf(x * x + y * y);
	else
		speed = sqrtf(x * x + y * y + z * z);

	w = tier_weight(p, speed);

	v[0] = do_filter(p, &ctx->ax, x, dt, forceSuppress, w, speed);
	v[1] = do_filter(p, &ctx->ay, y, dt, forceSuppress, w, speed);
	if (dim == 3)
		v[2] = do_filter(p, &ctx->az, z, dt, forceSuppress, w, speed);
}

#ifdef __cplusplus
}
#endif

#endif /* FAAL_IMPLEMENTATION */

#endif /* FAAL_H */