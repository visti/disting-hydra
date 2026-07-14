# Hydra

**[Read the manual](https://visti.github.io/disting-hydra/)**

CV-controlled **audio chord generator** for the Expert Sleepers **Disting NT** —
one head becomes many. Sibling to Chimera. Takes a single (stereo) input and
harmonises it into a chord *by audio*, pitch-shifting duplicates of the signal
onto notes of a scale or chord you define.

## What it does

Input is summed to mono into a ring buffer, then read back through 2–4
**granular pitch-shifters** (two-tap, Hann-windowed) — each a transposed copy of
the input. A **pitch tracker** measures the input's fundamental so each voice
lands on an *exact* target note (`voiceNote − inputNote`); turn **Track** off to
assume the input plays the **Root**.

### Pages

- **Scale** — 12 pitch-class toggles define the quantiser/scale, or pick a
  **Scale** preset (major, minor, modes, harmonic/melodic minor, pentatonics,
  whole-tone, chromatic). A 1V/oct **Pitch CV** picks the chord root, snapped to
  the scale. **Root** sets the reference note.
- **Harmony** — **Voicing**: *Scale-stack* (diatonic, stack by **Step**) or
  *Chord-type* (a chromatic table: maj, min, maj7, dom7, min7, sus2/4, dim, aug,
  6, add9, 5), with a **Chord CV** to select the type. **Voice lead** keeps each
  voice near its previous note (smoother, smaller shifts). **Bass** drops the
  lowest voice by 1–2 octaves.
- **Chord** — **Voices** (2–4), **Step**, **Inversion**, **Octave**, **Spread**
  (stereo pan), **Detune** (per-voice cents), **Glide** (portamento).
- **Motion** — **Freeze** (toggle **+ gate input**) sustains the buffer for
  infinite pads; **Smear** decorrelates the frozen grains into a phase-smeared
  cloud (PaulStretch-style) so it evolves instead of looping; **Strum** +
  **Strum dir** stagger the voices on a chord
  trigger (up/down/random); **Vib depth/rate** add a global chorus vibrato;
  **Env→Filter** lets input dynamics open the master filter.
- **Tone** — **Grain** size and **Grain sync** (snaps grains to the tracked
  period to cut warble); state-variable **Filter** (LP/BP/HP) with cutoff/res.
- **LPG** — per-voice low-pass gate (vactrol: envelope drives brightness + level)
  with **Attack**–**Decay** (long attack = swell) and **Depth**. Each voice is
  struck by **either** the chord **Trig** or its own **Trig 1–4**.
- **Outs** — per-voice individual audio **Out 1–4** (post-LPG; an assigned voice
  leaves the main mix), plus **Pitch 1–4** (1V/oct) and **Gate 1–4** CV outs so
  Hydra drives external oscillators with the same chord. **CV out oct** shifts
  the pitch-CV octave reference.
- **Routing** — input/output buses, output mode, mix, level.

## Building

Requires the [distingNT_API](https://github.com/expertsleepersltd/distingNT_API)
and the ARM GNU toolchain (`arm-none-eabi-c++`).

```sh
make            # builds plugins/hydra.o
make check      # host-side syntax check (clang++, no ARM toolchain needed)
```

`NT_API_PATH` defaults to `../distingNT_API`; override on the command line if it
lives elsewhere.

## Installing

Copy `plugins/hydra.o` to the SD card under `/programs/plug-ins/` and add the
algorithm on the NT.
