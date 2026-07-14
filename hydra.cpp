/*
 * Hydra
 * CV-controlled audio chord generator for the Expert Sleepers Disting NT.
 * One head becomes many: a single input, harmonised into a chord by audio.
 * Sibling to Chimera.
 *
 * Signal flow:
 *   stereo in -> mono -> ring buffer -> N granular pitch-shifters (2-4 voices)
 *   -> per-voice low-pass gate -> pan into main mix / individual outs
 *   -> master filter -> dry/wet -> level.
 *
 * A pitch tracker measures the input's fundamental so each voice is shifted
 * onto an exact scale note (voiceNote - inputNote); with Track off the input is
 * assumed to play the Root note. Voices are built either by stacking the scale
 * (Scale-stack) or from a chromatic chord-type table (Chord-type). The chord is
 * also emitted as per-voice pitch (1V/oct) and gate CV so Hydra can drive
 * external voices. Freeze sustains the buffer for pads; Strum staggers the
 * voices; a global vibrato and an input-driven filter add motion.
 */

#include <math.h>
#include <string.h>
#include <new>
#include <distingnt/api.h>
#include "hydra_glyph.h"

// ---------------------------------------------------------------------------
// constants

enum
{
	kMaxVoices	= 4,
	kBufLen		= 8192,			// mono input ring buffer (power of 2)
	kBufMask	= kBufLen - 1,
	kWinSize	= 1024,			// Hann window LUT
	kSinSize	= 256,			// vibrato sine LUT

	kDecim		= 4,			// pitch-analysis decimation factor
	kAnaLen		= 1024,			// analysis ring buffer (power of 2)
	kAnaMask	= kAnaLen - 1,
	kAnaWindow	= 400,			// autocorrelation window (decimated samples)
	kAnaHop		= 256,			// decimated samples between analyses
};

static const float kTrigHi = 1.0f;		// volts, rising-edge threshold
static const float kTrigLo = 0.5f;		// volts, re-arm threshold
static const float kGateHigh = 5.0f;	// volts, gate output high level
static const float kLn2over1200 = 0.69314718f / 1200.0f;

static inline int roundToInt( float x )
{
	return ( x >= 0.0f ) ? (int)( x + 0.5f ) : -(int)( -x + 0.5f );
}

// small xorshift RNG (for Random strum order)
struct Rng
{
	uint32_t s;
	void seed( uint32_t v ) { s = v ? v : 0x9e3779b9u; }
	uint32_t next() { uint32_t x=s; x^=x<<13; x^=x>>17; x^=x<<5; s=x; return x; }
	float uniform() { return ( next() >> 8 ) * ( 1.0f / 16777216.0f ); }	// [0,1)
};

// ---------------------------------------------------------------------------
// scales and chords

// pitch-class masks (bit i = pitch class i active), C-based. index 0 = Custom.
enum {
	kScaleCustom, kScaleMajor, kScaleMinor, kScaleDorian, kScalePhrygian,
	kScaleMixo, kScaleLydian, kScaleLocrian, kScaleHarmMin, kScaleMelMin,
	kScaleMajPent, kScaleMinPent, kScaleWhole, kScaleChromatic, kNumScales };
static const char* const scaleStrings[] = {
	"Custom", "Major", "Minor", "Dorian", "Phrygian", "Mixolydian", "Lydian",
	"Locrian", "Harm min", "Melodic min", "Maj pent", "Min pent", "Whole tone", "Chromatic" };
static const uint16_t scaleMask[ kNumScales ] = {
	0,			// Custom (unused; toggles used instead)
	0x0AB5,		// Major       1010 1011 0101
	0x05AD,		// Minor       0101 1010 1101
	0x06AD,		// Dorian
	0x05AB,		// Phrygian
	0x06B5,		// Mixolydian
	0x0AD5,		// Lydian
	0x056B,		// Locrian
	0x09AD,		// Harm minor
	0x0AAD,		// Melodic minor
	0x0295,		// Maj pentatonic  {0,2,4,7,9}
	0x04A9,		// Min pentatonic  {0,3,5,7,10}
	0x0555,		// Whole tone      {0,2,4,6,8,10}
	0x0FFF,		// Chromatic
};

// chromatic chord voicings: up to 4 intervals from the root
enum {
	kChMaj, kChMin, kChMaj7, kChDom7, kChMin7, kChSus2, kChSus4,
	kChDim, kChAug, kChSix, kChAdd9, kChFive, kNumChords };
static const char* const chordStrings[] = {
	"Maj", "Min", "Maj7", "Dom7", "Min7", "Sus2", "Sus4",
	"Dim", "Aug", "6", "add9", "5" };
static const int8_t chordIntervals[ kNumChords ][ kMaxVoices ] = {
	{ 0, 4, 7, 12 },	// Maj
	{ 0, 3, 7, 12 },	// Min
	{ 0, 4, 7, 11 },	// Maj7
	{ 0, 4, 7, 10 },	// Dom7
	{ 0, 3, 7, 10 },	// Min7
	{ 0, 2, 7, 12 },	// Sus2
	{ 0, 5, 7, 12 },	// Sus4
	{ 0, 3, 6, 9  },	// Dim
	{ 0, 4, 8, 12 },	// Aug
	{ 0, 4, 7, 9  },	// 6
	{ 0, 4, 7, 14 },	// add9
	{ 0, 7, 12, 19 },	// 5
};

// ---------------------------------------------------------------------------
// parameters

enum
{
	// routing / global
	kParamInputL,
	kParamInputR,
	kParamOutputL,
	kParamOutputMode,
	kParamOutputR,
	kParamPitchCV,
	kParamTrack,
	kParamMix,
	kParamLevel,

	// scale
	kParamScalePreset,
	kParamNote0, kParamNote1, kParamNote2, kParamNote3, kParamNote4, kParamNote5,
	kParamNote6, kParamNote7, kParamNote8, kParamNote9, kParamNote10, kParamNote11,
	kParamRoot,

	// harmony
	kParamVoicing,
	kParamChordType,
	kParamChordCV,
	kParamVoiceLead,
	kParamBass,

	// chord
	kParamVoices,
	kParamStep,
	kParamInversion,
	kParamOctave,
	kParamSpread,
	kParamDetune,
	kParamGlide,

	// motion
	kParamFreeze,
	kParamFreezeInput,
	kParamSmear,
	kParamStrum,
	kParamStrumDir,
	kParamVibDepth,
	kParamVibRate,
	kParamEnvFilter,

	// tone
	kParamGrain,
	kParamGrainSync,
	kParamFilterType,
	kParamCutoff,
	kParamResonance,

	// LPG
	kParamLpgMode,
	kParamAttack,
	kParamDecay,
	kParamLpgDepth,
	kParamTrig,
	kParamTrig1, kParamTrig2, kParamTrig3, kParamTrig4,

	// outputs
	kParamOut1, kParamOut2, kParamOut3, kParamOut4,
	kParamPitchOut1, kParamPitchOut2, kParamPitchOut3, kParamPitchOut4,
	kParamGateOut1, kParamGateOut2, kParamGateOut3, kParamGateOut4,
	kParamCVOutOct,

	kNumParams,
};

static const char* const offOnStrings[] = { "off", "on" };
static const char* const trackStrings[] = { "Off", "On" };
static const char* const filterTypeStrings[] = { "Off", "Low pass", "Band pass", "High pass" };
enum { kFilterOff, kFilterLP, kFilterBP, kFilterHP };
static const char* const lpgModeStrings[] = { "Off", "On" };
enum { kLpgOff, kLpgOn };
static const char* const voicingStrings[] = { "Scale-stack", "Chord-type" };
enum { kVoicingScale, kVoicingChord };
static const char* const bassStrings[] = { "Off", "-1 oct", "-2 oct" };
static const char* const strumDirStrings[] = { "Up", "Down", "Random" };
enum { kStrumUp, kStrumDown, kStrumRandom };

#define NOTE_PARAM( nm, on ) \
	{ .name = nm, .min = 0, .max = 1, .def = on, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = offOnStrings },

static const _NT_parameter	parameters[] = {
	NT_PARAMETER_AUDIO_INPUT( "Input L", 1, 1 )
	NT_PARAMETER_AUDIO_INPUT( "Input R", 1, 2 )
	NT_PARAMETER_AUDIO_OUTPUT_WITH_MODE( "Output L", 1, 13 )
	NT_PARAMETER_AUDIO_OUTPUT( "Output R", 1, 14 )
	NT_PARAMETER_CV_INPUT( "Pitch CV", 0, 0 )
	{ .name = "Track", .min = 0, .max = 1, .def = 1, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = trackStrings },
	{ .name = "Mix", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Level", .min = -40, .max = 6, .def = 0, .unit = kNT_unitDb, .scaling = 0, .enumStrings = NULL },

	{ .name = "Scale", .min = 0, .max = kNumScales - 1, .def = kScaleMajor, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = scaleStrings },
	NOTE_PARAM( "Note C",  1 )
	NOTE_PARAM( "Note C#", 0 )
	NOTE_PARAM( "Note D",  1 )
	NOTE_PARAM( "Note D#", 0 )
	NOTE_PARAM( "Note E",  1 )
	NOTE_PARAM( "Note F",  1 )
	NOTE_PARAM( "Note F#", 0 )
	NOTE_PARAM( "Note G",  1 )
	NOTE_PARAM( "Note G#", 0 )
	NOTE_PARAM( "Note A",  1 )
	NOTE_PARAM( "Note A#", 0 )
	NOTE_PARAM( "Note B",  1 )
	{ .name = "Root", .min = 0, .max = 127, .def = 48, .unit = kNT_unitMIDINote, .scaling = 0, .enumStrings = NULL },

	{ .name = "Voicing", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = voicingStrings },
	{ .name = "Chord type", .min = 0, .max = kNumChords - 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = chordStrings },
	NT_PARAMETER_CV_INPUT( "Chord CV", 0, 0 )
	{ .name = "Voice lead", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = offOnStrings },
	{ .name = "Bass", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = bassStrings },

	{ .name = "Voices", .min = 2, .max = kMaxVoices, .def = 3, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "Step", .min = 1, .max = 4, .def = 2, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "Inversion", .min = 0, .max = 3, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "Octave", .min = -2, .max = 2, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
	{ .name = "Spread", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Detune", .min = 0, .max = 50, .def = 0, .unit = kNT_unitCents, .scaling = 0, .enumStrings = NULL },
	{ .name = "Glide", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "Freeze", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = offOnStrings },
	NT_PARAMETER_CV_INPUT( "Freeze in", 0, 0 )
	{ .name = "Smear", .min = 0, .max = 100, .def = 60, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	{ .name = "Strum", .min = 0, .max = 500, .def = 0, .unit = kNT_unitMs, .scaling = 0, .enumStrings = NULL },
	{ .name = "Strum dir", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = strumDirStrings },
	{ .name = "Vib depth", .min = 0, .max = 50, .def = 0, .unit = kNT_unitCents, .scaling = 0, .enumStrings = NULL },
	{ .name = "Vib rate", .min = 1, .max = 120, .def = 50, .unit = kNT_unitHz, .scaling = kNT_scaling10, .enumStrings = NULL },
	{ .name = "Env->Filter", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "Grain", .min = 10, .max = 85, .def = 50, .unit = kNT_unitMs, .scaling = 0, .enumStrings = NULL },
	{ .name = "Grain sync", .min = 0, .max = 1, .def = 1, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = offOnStrings },
	{ .name = "Filter", .min = 0, .max = 3, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = filterTypeStrings },
	{ .name = "Cutoff", .min = 20, .max = 18000, .def = 8000, .unit = kNT_unitHz, .scaling = 0, .enumStrings = NULL },
	{ .name = "Resonance", .min = 0, .max = 100, .def = 10, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },

	{ .name = "LPG", .min = 0, .max = 1, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = lpgModeStrings },
	{ .name = "Attack", .min = 0, .max = 2000, .def = 5, .unit = kNT_unitMs, .scaling = 0, .enumStrings = NULL },
	{ .name = "Decay", .min = 5, .max = 4000, .def = 300, .unit = kNT_unitMs, .scaling = 0, .enumStrings = NULL },
	{ .name = "LPG depth", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
	NT_PARAMETER_CV_INPUT( "Trig", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Trig 1", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Trig 2", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Trig 3", 0, 0 )
	NT_PARAMETER_CV_INPUT( "Trig 4", 0, 0 )

	NT_PARAMETER_AUDIO_OUTPUT( "Out 1", 0, 0 )
	NT_PARAMETER_AUDIO_OUTPUT( "Out 2", 0, 0 )
	NT_PARAMETER_AUDIO_OUTPUT( "Out 3", 0, 0 )
	NT_PARAMETER_AUDIO_OUTPUT( "Out 4", 0, 0 )
	NT_PARAMETER_CV_OUTPUT( "Pitch 1", 0, 0 )
	NT_PARAMETER_CV_OUTPUT( "Pitch 2", 0, 0 )
	NT_PARAMETER_CV_OUTPUT( "Pitch 3", 0, 0 )
	NT_PARAMETER_CV_OUTPUT( "Pitch 4", 0, 0 )
	NT_PARAMETER_CV_OUTPUT( "Gate 1", 0, 0 )
	NT_PARAMETER_CV_OUTPUT( "Gate 2", 0, 0 )
	NT_PARAMETER_CV_OUTPUT( "Gate 3", 0, 0 )
	NT_PARAMETER_CV_OUTPUT( "Gate 4", 0, 0 )
	{ .name = "CV out oct", .min = -4, .max = 4, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
};

static const uint8_t pageScale[] = {
	kParamScalePreset,
	kParamNote0, kParamNote1, kParamNote2, kParamNote3, kParamNote4, kParamNote5,
	kParamNote6, kParamNote7, kParamNote8, kParamNote9, kParamNote10, kParamNote11, kParamRoot };
static const uint8_t pageHarmony[] = {
	kParamVoicing, kParamChordType, kParamChordCV, kParamVoiceLead, kParamBass };
static const uint8_t pageChord[] = {
	kParamVoices, kParamStep, kParamInversion, kParamOctave, kParamSpread, kParamDetune, kParamGlide };
static const uint8_t pageMotion[] = {
	kParamFreeze, kParamFreezeInput, kParamSmear, kParamStrum, kParamStrumDir, kParamVibDepth, kParamVibRate, kParamEnvFilter };
static const uint8_t pageTone[] = {
	kParamGrain, kParamGrainSync, kParamFilterType, kParamCutoff, kParamResonance };
static const uint8_t pageLpg[] = {
	kParamLpgMode, kParamAttack, kParamDecay, kParamLpgDepth, kParamTrig,
	kParamTrig1, kParamTrig2, kParamTrig3, kParamTrig4 };
static const uint8_t pageOuts[] = {
	kParamOut1, kParamOut2, kParamOut3, kParamOut4,
	kParamPitchOut1, kParamPitchOut2, kParamPitchOut3, kParamPitchOut4,
	kParamGateOut1, kParamGateOut2, kParamGateOut3, kParamGateOut4, kParamCVOutOct };
static const uint8_t pageRouting[] = {
	kParamInputL, kParamInputR, kParamOutputL, kParamOutputR, kParamOutputMode,
	kParamPitchCV, kParamMix, kParamLevel };

static const _NT_parameterPage pages[] = {
	{ .name = "Scale", .numParams = ARRAY_SIZE(pageScale), .params = pageScale },
	{ .name = "Harmony", .numParams = ARRAY_SIZE(pageHarmony), .params = pageHarmony },
	{ .name = "Chord", .numParams = ARRAY_SIZE(pageChord), .params = pageChord },
	{ .name = "Motion", .numParams = ARRAY_SIZE(pageMotion), .params = pageMotion },
	{ .name = "Tone", .numParams = ARRAY_SIZE(pageTone), .params = pageTone },
	{ .name = "LPG", .numParams = ARRAY_SIZE(pageLpg), .params = pageLpg },
	{ .name = "Outs", .numParams = ARRAY_SIZE(pageOuts), .params = pageOuts },
	{ .name = "Routing", .numParams = ARRAY_SIZE(pageRouting), .params = pageRouting },
};

static const _NT_parameterPages parameterPages = {
	.numPages = ARRAY_SIZE(pages),
	.pages = pages,
};

static const uint8_t voiceTrigParam[ kMaxVoices ]  = { kParamTrig1, kParamTrig2, kParamTrig3, kParamTrig4 };
static const uint8_t voiceOutParam[ kMaxVoices ]   = { kParamOut1, kParamOut2, kParamOut3, kParamOut4 };
static const uint8_t voicePitchParam[ kMaxVoices ] = { kParamPitchOut1, kParamPitchOut2, kParamPitchOut3, kParamPitchOut4 };
static const uint8_t voiceGateParam[ kMaxVoices ]  = { kParamGateOut1, kParamGateOut2, kParamGateOut3, kParamGateOut4 };

// ---------------------------------------------------------------------------
// algorithm

struct Voice
{
	float	ratio;			// current (glided) pitch ratio
	float	phase;			// grain sawtooth, [0,1)
	float	panL, panR;		// equal-power pan gains

	float	env;			// LPG attack-decay envelope
	int		stage;			// 0 idle, 1 attack, 2 decay
	float	lpgLp;			// vactrol one-pole state
	bool	armed;			// own-trigger edge-detect state

	float	strikeDelay;	// strum: output frames until this voice strikes
	bool	strikePending;

	// smeared freeze: each grain (tap) reads from a random buffer offset,
	// re-chosen when that tap wraps, decorrelating the frozen loop into a cloud
	float	offA, offB;		// random read offsets (frames)
	float	prevFa, prevFb;	// previous tap phases, for wrap detection
};

struct _hydra : public _NT_algorithm
{
	_hydra() {}
	~_hydra() {}

	float*		buf;			// DRAM: mono input ring buffer
	uint32_t	writePos;

	Voice		voice[ kMaxVoices ];
	float		targetRatio[ kMaxVoices ];
	int			voiceNote[ kMaxVoices ];	// absolute target note (for CV out)
	int			prevVoiceNote[ kMaxVoices ];// for voice-leading
	bool		voiceInit;

	// cached coefficients
	float		grainLen, invGrain;
	float		glideCoef;
	float		wetGain, dryGain, levelGain;
	int			filterType;
	float		fCoef, fDamp;
	int			lpgMode;
	float		attackCoef, decayCoef, lpgDepth;
	float		lpgCutMin, lpgCutMax;
	bool		chordArmed;
	bool		trackOn, grainSync;
	float		vibDepthCents, lfoInc;
	float		envFilterAmt, envFollowCoef;
	float		vu, vuCoef;			// smoothed output level, for the screen

	// filter + follower state
	float		svLowL, svBandL, svLowR, svBandR;
	float		envFollow;
	float		lfoPhase;

	// pitch tracking
	float*		abuf;
	uint32_t	anaWrite;
	int			decCount;
	float		decAccum;
	int			sinceAna;
	int			lagMin, lagMax;
	float		anaFs;
	float		trackedNote, trackedPeriod;	// period in host frames
	bool		trackedValid;

	// display
	int			dispRoot, dispVoices;

	Rng			rng;
	float		win[ kWinSize ];
	float		sinLut[ kSinSize ];
};

// ---------------------------------------------------------------------------
// scale helpers

static inline bool noteActive( _hydra* t, int pc )
{
	pc %= 12;
	if ( pc < 0 ) pc += 12;
	int preset = t->v[ kParamScalePreset ];
	if ( preset != kScaleCustom )
		return ( scaleMask[ preset ] >> pc ) & 1;
	return t->v[ kParamNote0 + pc ] != 0;
}

static bool anyNoteActive( _hydra* t )
{
	if ( t->v[ kParamScalePreset ] != kScaleCustom )
		return true;
	for ( int i=0; i<12; ++i )
		if ( t->v[ kParamNote0 + i ] )
			return true;
	return false;
}

static int snapNearest( _hydra* t, int note )
{
	if ( noteActive( t, note ) )
		return note;
	for ( int r=1; r<=6; ++r )
	{
		if ( noteActive( t, note - r ) ) return note - r;
		if ( noteActive( t, note + r ) ) return note + r;
	}
	return note;
}

static int walkUp( _hydra* t, int note, int steps )
{
	for ( int s=0; s<steps; ++s )
	{
		int guard = 0;
		do { ++note; } while ( !noteActive( t, note ) && ++guard < 24 );
	}
	return note;
}

// ---------------------------------------------------------------------------
// pitch tracking

static float autocorr( const float* abuf, uint32_t end, int W, int lag )
{
	float acc = 0.0f;
	for ( int k=0; k<W; ++k )
		acc += abuf[ ( end - 1 - k ) & kAnaMask ] * abuf[ ( end - 1 - k - lag ) & kAnaMask ];
	return acc;
}

static void detectPitch( _hydra* t )
{
	const float* abuf = t->abuf;
	uint32_t end = t->anaWrite;
	int W = kAnaWindow;

	float r0 = autocorr( abuf, end, W, 0 );
	if ( r0 < 1e-5f )
		return;

	float best = 0.0f;
	int bestLag = 0;
	for ( int lag = t->lagMin; lag <= t->lagMax; ++lag )
	{
		float c = autocorr( abuf, end, W, lag );
		if ( c > best ) { best = c; bestLag = lag; }
	}
	if ( bestLag == 0 || best < 0.4f * r0 )
		return;

	float lagF = bestLag;
	if ( bestLag > t->lagMin && bestLag < t->lagMax )
	{
		float cm = autocorr( abuf, end, W, bestLag - 1 );
		float cp = autocorr( abuf, end, W, bestLag + 1 );
		float denom = cm - 2.0f * best + cp;
		if ( denom != 0.0f )
			lagF = bestLag + 0.5f * ( cm - cp ) / denom;
	}

	float freq = t->anaFs / lagF;
	float note = 69.0f + 12.0f * log2f( freq / 440.0f );
	if ( note < 0.0f ) note = 0.0f;
	if ( note > 127.0f ) note = 127.0f;
	t->trackedNote += ( note - t->trackedNote ) * 0.4f;
	t->trackedPeriod = (float)NT_globals.sampleRate / freq;	// host frames
	t->trackedValid = true;
}

// ---------------------------------------------------------------------------
// construction

void	calculateRequirements( _NT_algorithmRequirements& req, const int32_t* specifications )
{
	(void)specifications;
	req.numParameters = kNumParams;
	req.sram = sizeof(_hydra);
	req.dram = ( kBufLen + kAnaLen ) * sizeof(float);
	req.dtc = 0;
	req.itc = 0;
}

_NT_algorithm*	construct( const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements& req, const int32_t* specifications )
{
	(void)req;
	(void)specifications;
	_hydra* alg = new (ptrs.sram) _hydra();
	alg->parameters = parameters;
	alg->parameterPages = &parameterPages;

	alg->buf = (float*)ptrs.dram;
	alg->abuf = alg->buf + kBufLen;
	memset( alg->buf, 0, ( kBufLen + kAnaLen ) * sizeof(float) );

	for ( int v=0; v<kMaxVoices; ++v )
	{
		alg->voice[v].ratio = 1.0f;
		alg->voice[v].phase = v * ( 1.0f / kMaxVoices );
		alg->voice[v].panL = alg->voice[v].panR = 0.70710678f;
		alg->voice[v].armed = true;
		alg->voice[v].offA = alg->voice[v].offB = 0.0f;
		alg->voice[v].prevFa = alg->voice[v].prevFb = 0.0f;
		alg->targetRatio[v] = 1.0f;
	}
	alg->chordArmed = true;
	alg->voiceInit = false;

	alg->anaFs = (float)NT_globals.sampleRate / kDecim;
	alg->lagMin = (int)( alg->anaFs / 1000.0f );
	alg->lagMax = (int)( alg->anaFs / 60.0f );
	if ( alg->lagMax > kAnaLen - kAnaWindow - 2 )
		alg->lagMax = kAnaLen - kAnaWindow - 2;
	alg->trackedNote = 60.0f;
	alg->trackedPeriod = 0.0f;
	alg->vu = 0.0f;
	alg->vuCoef = 1.0f - expf( -1.0f / ( (float)NT_globals.sampleRate * 0.05f ) );

	for ( int i=0; i<kWinSize; ++i )
		alg->win[i] = 0.5f - 0.5f * cosf( 6.2831853f * i / kWinSize );
	for ( int i=0; i<kSinSize; ++i )
		alg->sinLut[i] = sinf( 6.2831853f * i / kSinSize );

	alg->rng.seed( 0x48594452u );	// "HYDR"
	return alg;
}

// ---------------------------------------------------------------------------
// parameters

static void updateGrayedOut( _hydra* pThis )
{
	int algIdx = NT_algorithmIndex( pThis );
	if ( algIdx < 0 )
		return;
	uint32_t off = NT_parameterOffset();
	bool custom = ( pThis->v[ kParamScalePreset ] == kScaleCustom );
	for ( int i=0; i<12; ++i )
		NT_setParameterGrayedOut( algIdx, kParamNote0 + i + off, !custom );
	bool chordMode = ( pThis->v[ kParamVoicing ] == kVoicingChord );
	NT_setParameterGrayedOut( algIdx, kParamChordType + off, !chordMode );
	NT_setParameterGrayedOut( algIdx, kParamChordCV + off, !chordMode );
	NT_setParameterGrayedOut( algIdx, kParamStep + off, chordMode );
}

static void updateCoeffs( _hydra* pThis )
{
	float fs = (float)NT_globals.sampleRate;

	pThis->grainLen = pThis->v[ kParamGrain ] * fs / 1000.0f;
	if ( pThis->grainLen > kBufLen - 256 ) pThis->grainLen = kBufLen - 256;
	if ( pThis->grainLen < 32.0f ) pThis->grainLen = 32.0f;
	pThis->invGrain = 1.0f / pThis->grainLen;

	float g = pThis->v[ kParamGlide ] / 100.0f;
	float tau = g * g * 0.5f;
	pThis->glideCoef = ( tau > 0.0f ) ? 1.0f - expf( -1.0f / ( fs * tau ) ) : 1.0f;

	float mix = pThis->v[ kParamMix ] / 100.0f;
	pThis->wetGain = mix;
	pThis->dryGain = 1.0f - mix;
	pThis->levelGain = powf( 10.0f, pThis->v[ kParamLevel ] / 20.0f );

	pThis->filterType = pThis->v[ kParamFilterType ];
	float f = 2.0f * sinf( 3.14159265f * pThis->v[ kParamCutoff ] / fs );
	if ( f > 1.0f ) f = 1.0f;
	pThis->fCoef = f;
	pThis->fDamp = 2.0f - 1.95f * ( pThis->v[ kParamResonance ] / 100.0f );

	pThis->lpgMode = pThis->v[ kParamLpgMode ];
	float tauA = pThis->v[ kParamAttack ] / 1000.0f;
	pThis->attackCoef = ( tauA > 0.0f ) ? 1.0f - expf( -1.0f / ( fs * tauA ) ) : 1.0f;
	float tauD = pThis->v[ kParamDecay ] / 1000.0f;
	pThis->decayCoef = expf( -1.0f / ( fs * tauD ) );
	pThis->lpgDepth = pThis->v[ kParamLpgDepth ] / 100.0f;
	float twoPiOverFs = 6.2831853f / fs;
	pThis->lpgCutMin = twoPiOverFs * 150.0f;
	float cMax = twoPiOverFs * 10000.0f;
	pThis->lpgCutMax = ( cMax > 1.0f ) ? 1.0f : cMax;

	pThis->trackOn = pThis->v[ kParamTrack ] != 0;
	pThis->grainSync = pThis->v[ kParamGrainSync ] != 0;
	pThis->vibDepthCents = (float)pThis->v[ kParamVibDepth ];
	pThis->lfoInc = ( pThis->v[ kParamVibRate ] / 10.0f ) / fs;	// scaling10 -> Hz
	pThis->envFilterAmt = pThis->v[ kParamEnvFilter ] / 100.0f;
	pThis->envFollowCoef = 1.0f - expf( -1.0f / ( fs * 0.02f ) );	// 20ms follower
	pThis->vuCoef = 1.0f - expf( -1.0f / ( fs * 0.05f ) );			// 50ms screen meter
}

void	parameterChanged( _NT_algorithm* self, int p )
{
	(void)p;
	_hydra* pThis = (_hydra*)self;
	updateCoeffs( pThis );
	updateGrayedOut( pThis );
}

// ---------------------------------------------------------------------------
// per-block chord computation

static void computeChord( _hydra* pThis, const float* busFrames, int numFrames )
{
	float cv = 0.0f;
	int cvBus = pThis->v[ kParamPitchCV ];
	if ( cvBus > 0 )
		cv = busFrames[ ( cvBus - 1 ) * numFrames ];

	int base = pThis->v[ kParamRoot ];
	int rawTarget = base + roundToInt( cv * 12.0f );
	bool anyActive = anyNoteActive( pThis );
	int rootNote = anyActive ? snapNearest( pThis, rawTarget ) : rawTarget;

	float inputNote = pThis->trackOn ? pThis->trackedNote : (float)base;

	int voices = pThis->v[ kParamVoices ];
	int step   = pThis->v[ kParamStep ];
	int inv    = pThis->v[ kParamInversion ];
	int octave = pThis->v[ kParamOctave ];
	int bass   = pThis->v[ kParamBass ];		// 0 off, 1 -1oct, 2 -2oct
	bool chordMode = ( pThis->v[ kParamVoicing ] == kVoicingChord );
	bool voiceLead = pThis->v[ kParamVoiceLead ] != 0;
	float detune = pThis->v[ kParamDetune ];
	float spread = pThis->v[ kParamSpread ] / 100.0f;

	int chordType = pThis->v[ kParamChordType ];
	int ccBus = pThis->v[ kParamChordCV ];
	if ( chordMode && ccBus > 0 )
		chordType += roundToInt( busFrames[ ( ccBus - 1 ) * numFrames ] * 2.0f );	// ~0.5V/chord
	if ( chordType < 0 ) chordType = 0;
	if ( chordType >= kNumChords ) chordType = kNumChords - 1;

	pThis->dispRoot = rootNote;
	pThis->dispVoices = voices;

	for ( int v=0; v<voices; ++v )
	{
		int note;
		if ( bass && v == 0 )
			note = rootNote - 12 * bass;			// bass takes the lowest voice
		else if ( chordMode )
			note = rootNote + chordIntervals[ chordType ][ v ] + 12 * octave;
		else
		{
			note = anyActive ? walkUp( pThis, rootNote, v * step ) : rootNote + v * step;
			if ( v < inv ) note += 12;
			note += 12 * octave;
		}

		// voice-leading: keep near the previous note (skip the bass voice)
		if ( voiceLead && pThis->voiceInit && !( bass && v == 0 ) )
		{
			int pv = pThis->prevVoiceNote[v];
			while ( note - pv > 6 ) note -= 12;
			while ( pv - note > 6 ) note += 12;
		}

		pThis->voiceNote[v] = note;

		float semi = (float)note - inputNote;
		if ( v > 0 )
		{
			float dir = ( v & 1 ) ? 1.0f : -1.0f;
			semi += dir * detune * ( ( v + 1 ) / 2 ) / 100.0f;
		}
		pThis->targetRatio[v] = powf( 2.0f, semi / 12.0f );

		float pos = ( voices > 1 ) ? ( (float)v / ( voices - 1 ) * 2.0f - 1.0f ) * spread : 0.0f;
		float ang = ( pos + 1.0f ) * 0.25f * 3.14159265f;
		pThis->voice[v].panL = cosf( ang );
		pThis->voice[v].panR = sinf( ang );
	}
	for ( int v=0; v<voices; ++v )
		pThis->prevVoiceNote[v] = pThis->voiceNote[v];
	pThis->voiceInit = true;
}

// ---------------------------------------------------------------------------
// audio

static inline float readInterp( const float* buf, uint32_t wi, float d )
{
	int id = (int)d;
	float fr = d - id;
	uint32_t i0 = ( wi - id ) & kBufMask;
	uint32_t i1 = ( i0 - 1 ) & kBufMask;
	return buf[i0] * ( 1.0f - fr ) + buf[i1] * fr;
}

void 	step( _NT_algorithm* self, float* busFrames, int numFramesBy4 )
{
	_hydra* pThis = (_hydra*)self;
	int numFrames = numFramesBy4 * 4;

	computeChord( pThis, busFrames, numFrames );

	const float* inL  = busFrames + ( pThis->v[ kParamInputL  ] - 1 ) * numFrames;
	const float* inR  = busFrames + ( pThis->v[ kParamInputR  ] - 1 ) * numFrames;
	float*       outL = busFrames + ( pThis->v[ kParamOutputL ] - 1 ) * numFrames;
	float*       outR = busFrames + ( pThis->v[ kParamOutputR ] - 1 ) * numFrames;

	int voices = pThis->v[ kParamVoices ];
	bool replace = pThis->v[ kParamOutputMode ];
	int lpgMode = pThis->lpgMode;
	float attackCoef = pThis->attackCoef, decayCoef = pThis->decayCoef, lpgDepth = pThis->lpgDepth;
	float lpgCutMin = pThis->lpgCutMin, lpgCutSpan = pThis->lpgCutMax - pThis->lpgCutMin;
	bool trackOn = pThis->trackOn;
	float wet = pThis->wetGain, dry = pThis->dryGain, level = pThis->levelGain;
	float voiceScale = 1.0f / sqrtf( (float)voices );
	int fType = pThis->filterType;
	float fCoef = pThis->fCoef, fDamp = pThis->fDamp;
	float glide = pThis->glideCoef;
	float vuCoef = pThis->vuCoef;
	float vibScale = pThis->vibDepthCents * kLn2over1200;
	float lfoInc = pThis->lfoInc;
	float envFilterAmt = pThis->envFilterAmt, envFollowCoef = pThis->envFollowCoef;

	// grain length (optionally snapped to the tracked period)
	float G = pThis->grainLen;
	if ( pThis->grainSync && trackOn && pThis->trackedValid && pThis->trackedPeriod > 1.0f )
	{
		int m = roundToInt( pThis->grainLen / pThis->trackedPeriod );
		if ( m < 1 ) m = 1;
		G = pThis->trackedPeriod * m;
		if ( G > kBufLen - 256 ) G = kBufLen - 256;
	}
	float invG = 1.0f / G;

	// freeze while the param is on OR the freeze gate is held high
	bool freeze = pThis->v[ kParamFreeze ] != 0;
	int fzBus = pThis->v[ kParamFreezeInput ];
	if ( fzBus > 0 && busFrames[ ( fzBus - 1 ) * numFrames ] >= kTrigHi )
		freeze = true;

	// smeared-freeze: max random read offset (frames) spread across the buffer
	float maxOff = ( pThis->v[ kParamSmear ] / 100.0f ) * ( kBufLen - G - 2.0f );
	if ( maxOff < 0.0f ) maxOff = 0.0f;

	// trigger + output bus pointers
	const float* chordTrig = NULL;
	const float* vTrig[ kMaxVoices ] = { NULL, NULL, NULL, NULL };
	float* vOut[ kMaxVoices ]   = { NULL, NULL, NULL, NULL };
	float* vPitch[ kMaxVoices ] = { NULL, NULL, NULL, NULL };
	float* vGate[ kMaxVoices ]  = { NULL, NULL, NULL, NULL };
	if ( lpgMode == kLpgOn )
	{
		int b = pThis->v[ kParamTrig ];
		if ( b > 0 ) chordTrig = busFrames + ( b - 1 ) * numFrames;
	}
	for ( int v=0; v<voices; ++v )
	{
		if ( lpgMode == kLpgOn )
		{
			int tb = pThis->v[ voiceTrigParam[v] ];
			if ( tb > 0 ) vTrig[v] = busFrames + ( tb - 1 ) * numFrames;
		}
		int ob = pThis->v[ voiceOutParam[v] ];
		if ( ob > 0 ) vOut[v] = busFrames + ( ob - 1 ) * numFrames;
		int pb = pThis->v[ voicePitchParam[v] ];
		if ( pb > 0 ) vPitch[v] = busFrames + ( pb - 1 ) * numFrames;
		int gb = pThis->v[ voiceGateParam[v] ];
		if ( gb > 0 ) vGate[v] = busFrames + ( gb - 1 ) * numFrames;
	}

	// per-voice pitch CV out value (block-rate): 1V/oct, C1 (24) = 0V + offset
	float cvOctOff = (float)pThis->v[ kParamCVOutOct ];
	for ( int v=0; v<voices; ++v )
		if ( vPitch[v] )
		{
			float volts = ( pThis->voiceNote[v] - 24 ) / 12.0f + cvOctOff;
			for ( int i=0; i<numFrames; ++i )
				vPitch[v][i] = volts;
		}

	// strum timing
	float strumFrames = pThis->v[ kParamStrum ] * (float)NT_globals.sampleRate / 1000.0f;
	int strumDir = pThis->v[ kParamStrumDir ];

	float* buf = pThis->buf;
	bool pendingAnalysis = false;

	for ( int i=0; i<numFrames; ++i )
	{
		float dryL = inL[i], dryR = inR[i];
		float mono = 0.5f * ( dryL + dryR );

		// write the delay line first; the read taps below reference this index
		// (delay 0 = current sample). Frozen: stop writing so the taps sweep a
		// static buffer, sustaining the chord.
		uint32_t wi = pThis->writePos & kBufMask;
		if ( !freeze )
		{
			buf[ wi ] = mono;
			if ( trackOn )
			{
				pThis->decAccum += mono;
				if ( ++pThis->decCount >= kDecim )
				{
					pThis->abuf[ pThis->anaWrite & kAnaMask ] = pThis->decAccum * ( 1.0f / kDecim );
					pThis->anaWrite++;
					pThis->decCount = 0;
					pThis->decAccum = 0.0f;
					if ( ++pThis->sinceAna >= kAnaHop ) { pThis->sinceAna = 0; pendingAnalysis = true; }
				}
			}
		}

		// input envelope follower (for Env->Filter)
		float a = mono < 0.0f ? -mono : mono;
		pThis->envFollow += ( a - pThis->envFollow ) * envFollowCoef;
		float fCoefEff = fCoef;
		if ( envFilterAmt > 0.0f )
		{
			fCoefEff = fCoef * ( 1.0f + pThis->envFollow * envFilterAmt * 4.0f );
			if ( fCoefEff > 1.0f ) fCoefEff = 1.0f;
		}

		// vibrato LFO
		pThis->lfoPhase += lfoInc;
		if ( pThis->lfoPhase >= 1.0f ) pThis->lfoPhase -= 1.0f;

		// chord trigger edge -> schedule strum
		if ( lpgMode == kLpgOn && chordTrig )
		{
			float t = chordTrig[i];
			if ( pThis->chordArmed && t >= kTrigHi )
			{
				pThis->chordArmed = false;
				for ( int slot=0; slot<voices; ++slot )
				{
					int vv = ( strumDir == kStrumDown ) ? ( voices - 1 - slot ) : slot;
					if ( strumDir == kStrumRandom )
						vv = pThis->rng.next() % voices;
					pThis->voice[vv].strikeDelay = slot * strumFrames;
					pThis->voice[vv].strikePending = true;
				}
			}
			else if ( !pThis->chordArmed && t < kTrigLo )
				pThis->chordArmed = true;
		}

		float wetL = 0.0f, wetR = 0.0f;
		for ( int v=0; v<voices; ++v )
		{
			Voice& vc = pThis->voice[v];
			vc.ratio += ( pThis->targetRatio[v] - vc.ratio ) * glide;

			// vibrato (per-voice phase offset for chorus)
			float lfoV = pThis->sinLut[ ( (int)( pThis->lfoPhase * kSinSize ) + v * ( kSinSize / kMaxVoices ) ) & ( kSinSize - 1 ) ];
			float r = vc.ratio * ( 1.0f + vibScale * lfoV );

			float ph = vc.phase + ( 1.0f - r ) * invG;
			ph -= (int)ph;
			if ( ph < 0.0f ) ph += 1.0f;
			vc.phase = ph;

			float fa = ph;
			float fb = ph + 0.5f;
			if ( fb >= 1.0f ) fb -= 1.0f;

			// smeared freeze: when a grain (tap) wraps, jump it to a new random
			// position in the frozen buffer so successive grains decorrelate
			float dA = fa * G, dB = fb * G;
			if ( freeze && maxOff > 0.0f )
			{
				if ( fabsf( fa - vc.prevFa ) > 0.5f ) vc.offA = pThis->rng.uniform() * maxOff;
				if ( fabsf( fb - vc.prevFb ) > 0.5f ) vc.offB = pThis->rng.uniform() * maxOff;
				dA += vc.offA;
				dB += vc.offB;
			}
			vc.prevFa = fa;
			vc.prevFb = fb;

			float sa = readInterp( buf, wi, dA );
			float sb = readInterp( buf, wi, dB );
			float s = pThis->win[ (int)( fa * kWinSize ) ] * sa
			        + pThis->win[ (int)( fb * kWinSize ) ] * sb;

			// LPG: strum-scheduled strike, own-trigger strike, attack-decay
			if ( lpgMode == kLpgOn )
			{
				bool strike = false;
				if ( vc.strikePending )
				{
					if ( vc.strikeDelay <= 0.0f ) { strike = true; vc.strikePending = false; }
					else vc.strikeDelay -= 1.0f;
				}
				if ( vTrig[v] )
				{
					float t = vTrig[v][i];
					if ( vc.armed && t >= kTrigHi ) { strike = true; vc.armed = false; }
					else if ( !vc.armed && t < kTrigLo ) vc.armed = true;
				}
				if ( strike )
				{
					vc.stage = 1;
					if ( attackCoef >= 1.0f ) { vc.env = 1.0f; vc.stage = 2; }
				}
				if ( vc.stage == 1 )
				{
					vc.env += ( 1.0f - vc.env ) * attackCoef;
					if ( vc.env >= 0.999f ) { vc.env = 1.0f; vc.stage = 2; }
				}
				else if ( vc.stage == 2 )
				{
					vc.env *= decayCoef;
					if ( vc.env < 1e-4f ) { vc.env = 0.0f; vc.stage = 0; }
				}
				float env = vc.env;
				float cut = lpgCutMin + lpgCutSpan * env;
				vc.lpgLp += cut * ( s - vc.lpgLp );
				float gated = vc.lpgLp * env;
				s += ( gated - s ) * lpgDepth;
			}

			// gate output: high while sounding (LPG env, or always on if LPG off)
			if ( vGate[v] )
			{
				float g5 = ( lpgMode == kLpgOn ) ? ( vc.env > 0.01f ? kGateHigh : 0.0f ) : kGateHigh;
				vGate[v][i] = g5;
			}

			if ( vOut[v] )
				vOut[v][i] += s * level;
			else
			{
				wetL += s * vc.panL;
				wetR += s * vc.panR;
			}
		}
		wetL *= voiceScale;
		wetR *= voiceScale;
		pThis->vu += ( 0.5f * ( fabsf(wetL) + fabsf(wetR) ) - pThis->vu ) * vuCoef;

		if ( fType != kFilterOff )
		{
			float hiL = wetL - pThis->svLowL - fDamp * pThis->svBandL;
			pThis->svBandL += fCoefEff * hiL;
			pThis->svLowL  += fCoefEff * pThis->svBandL;
			float hiR = wetR - pThis->svLowR - fDamp * pThis->svBandR;
			pThis->svBandR += fCoefEff * hiR;
			pThis->svLowR  += fCoefEff * pThis->svBandR;
			switch ( fType )
			{
			case kFilterLP: wetL = pThis->svLowL;  wetR = pThis->svLowR;  break;
			case kFilterBP: wetL = pThis->svBandL; wetR = pThis->svBandR; break;
			case kFilterHP: wetL = hiL;            wetR = hiR;            break;
			}
		}

		float l = ( dry * dryL + wet * wetL ) * level;
		float r = ( dry * dryR + wet * wetR ) * level;
		if ( replace ) { outL[i] = l; outR[i] = r; }
		else           { outL[i] += l; outR[i] += r; }

		if ( !freeze )
			pThis->writePos++;
	}

	if ( pendingAnalysis )
		detectPitch( pThis );
}

// ---------------------------------------------------------------------------
// display

// write one 4-bit pixel into the screen buffer (two pixels per byte)
static inline void hydraSetPix( int x, int y, int v )
{
	if ( x < 0 || y < 0 || x >= 256 || y >= 64 ) return;
	int idx = y * 128 + ( x >> 1 );
	uint8_t b = NT_screen[ idx ];
	if ( x & 1 ) b = ( b & 0xF0 ) | ( v & 0x0F );
	else         b = ( b & 0x0F ) | ( ( v & 0x0F ) << 4 );
	NT_screen[ idx ] = b;
}

// append a note name + octave (e.g. "C3") to buff at position n; returns new n
static int appendNote( char* buff, int n, int note )
{
	static const char* const names[12] = { "C","C#","D","D#","E","F","F#","G","G#","A","A#","B" };
	int pc = note % 12; if ( pc < 0 ) pc += 12;
	for ( const char* p=names[pc]; *p; ++p ) buff[n++] = *p;
	char ob[8]; int ol = NT_intToString( ob, note / 12 - 1 );
	for ( int k=0; k<ol; ++k ) buff[n++] = ob[k];
	return n;
}

bool	draw( _NT_algorithm* self )
{
	_hydra* pThis = (_hydra*)self;

	// the hydra, on the right, its brightness pulsing with the output level
	float lvl = pThis->vu * 3.0f; if ( lvl > 1.0f ) lvl = 1.0f;
	float bscale = 0.42f + 0.58f * lvl;
	int X0 = 256 - kHydraGlyphW - 2;
	int Y0 = ( 64 - kHydraGlyphH ) / 2;
	for ( int gy=0; gy<kHydraGlyphH; ++gy )
		for ( int gx=0; gx<kHydraGlyphW; ++gx )
		{
			uint8_t L = kHydraGlyph[ gy*kHydraGlyphW + gx ];
			if ( !L ) continue;
			int b = (int)( L * bscale + 0.5f );
			if ( b < 1 ) b = 1;
			if ( b > 15 ) b = 15;
			hydraSetPix( X0+gx, Y0+gy, b );
		}

	// readouts on the left
	char buff[ 24 ];
	int n = 0;
	buff[n++]='R'; buff[n++]=' ';
	n = appendNote( buff, n, pThis->dispRoot );
	buff[n]=0;
	NT_drawText( 4, 12, buff );

	if ( pThis->trackOn )
	{
		n = 0; buff[n++]='I'; buff[n++]='n'; buff[n++]=' ';
		n = appendNote( buff, n, roundToInt( pThis->trackedNote ) );
		buff[n]=0;
		NT_drawText( 4, 26, buff );
	}

	// per-voice level meters (the many heads): LPG envelope, or full when LPG off
	bool lpgOn = pThis->v[ kParamLpgMode ] == kLpgOn;
	NT_drawText( 4, 38, "voices" );
	for ( int v=0; v<pThis->dispVoices; ++v )
	{
		float e = lpgOn ? pThis->voice[v].env : 1.0f;
		if ( e < 0.0f ) e = 0.0f;
		if ( e > 1.0f ) e = 1.0f;
		int x = 4 + v*10;
		int h = 1 + (int)( e * 18.0f );
		NT_drawShapeI( kNT_box, x, 42, x+6, 61, 3 );			// meter frame
		NT_drawShapeI( kNT_rectangle, x, 61-h, x+6, 61, 13 );	// level fill
	}

	return false;
}

// ---------------------------------------------------------------------------
// factory

static const _NT_factory factory =
{
	.guid = NT_MULTICHAR( 'H', 'y', 'd', 'r' ),
	.name = "Hydra",
	.description = "Audio chord generator / pitch-tracking harmoniser",
	.numSpecifications = 0,
	.specifications = NULL,
	.calculateStaticRequirements = NULL,
	.initialise = NULL,
	.calculateRequirements = calculateRequirements,
	.construct = construct,
	.parameterChanged = parameterChanged,
	.step = step,
	.draw = draw,
	.tags = kNT_tagEffect,
	.parameterString = NULL,
};

uintptr_t pluginEntry( _NT_selector selector, uint32_t data )
{
	switch ( selector )
	{
	case kNT_selector_version:
		return kNT_apiVersionCurrent;
	case kNT_selector_numFactories:
		return 1;
	case kNT_selector_factoryInfo:
		return (uintptr_t)( ( data == 0 ) ? &factory : NULL );
	}
	return 0;
}
