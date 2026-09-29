// SPDX-License-Identifier: MIT
#ifndef MAGLOCATOR_H
#define MAGLOCATOR_H
// ---------------------------------------------------------------------------
// Turns MagArray frames into "where is the probe": presence, magnet position,
// which way it points, and where the probe TIP is (the magnet sits some way up
// the probe's axis from the tip, and the fit gives that axis for free).
//
// Runs one dipole fit (MagFit) per new frame, warm-started from the last fix.
//
// Console: d = stream fixes as CSV, l = one-line summary of the latest fix,
// o = orientation check (which mounting of the sensors explains a held magnet),
// k = hold the magnet's strength and measure it again from scratch, K = fit it freely,
// t = the magnet's centre is this far up the probe from its point (t12<Enter>),
// T = the magnet's angle to the shaft (T90<Enter> = a disc lying flat on it),
// u = cursor under the tip / where it points, S = surface height, g = tracker on/off.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "MagFit.h"
#include "MagTracker.h"
#include "SurfaceMap.h"
#include "TipModel.h"
#include "ZeroAudit.h"
#include "MagOffsetFilter.h"
#include "Vec3.h"

// Weak magnets. One frame carries about 0.011 mT of noise per axis (measured), and a small
// magnet two sensor pitches away is not much more than that, so the fields
// are smoothed before anything looks at them - heavily when the strongest
// reading is weak, not at all once it passes MAGLOC_FAST_MT, so a strong magnet
// still tracks without lag. Smoothed noise is about 0.005 mT. (A deeper
// floor is the far probe's cure - its noise is the sensors', and only
// averaging longer helps: at 0.03 the sim's fix 45 mm up scattered +/-2.4 mm
// and +/-5 degrees for 0.08's +/-4 and +/-8, at a 0.33 s lag far up - but
// tried on 2026-09-28 evening the bench-like session's zero audit then took
// three gains for off that were not, and it went back to 0.08 until that is
// understood.)
#define MAGLOC_FAST_MT 2.0f        // strongest reading at which smoothing is off
#define MAGLOC_SLOWEST_ALPHA 0.08f // smoothing at the weak end (~0.12 s time constant at 100 Hz)
// ...and off again while the probe MOVES: smoothed fields lag, and a fix
// made of lagging fields is precise about where the probe was 100 ms ago
// (8 mm behind a hand at 80 mm/s, in simulation, with a 0.7 mm bar). The
// tracker knows the speed, so the smoothing backs off with it: alpha is at
// least speed x this (1 = none at 50 mm/s); at rest, where noise matters
// and lag does not, the weak-signal smoothing is back.
#define MAGLOC_ALPHA_PER_MM_S 0.02f
// The speed for that rule is the larger of two, each less what jitter alone
// would make of it: the distance the track moved over the last
// MAGLOC_SPEED_WINDOW frames, over that time, and the filter's own velocity.
// That velocity is also the fix jitter over one 10 ms frame - 10-20 mm/s on
// a probe at rest near the board (measured 2026-09-19), 50 on one far up -
// and taken as the speed with a fixed 5 mm/s off it opened the smoothing on
// any resting probe (a far one to alpha 0.6 in pencil.cpp: the smoothing
// defeated by its own noise, 0.9 mm of jitter at rest). Jitter's share of
// each speed is MAGLOC_SPEED_JITTER_K x the fix's own error bar over the
// time; 0.4 keeps a far probe at rest to 0.13 mm (from 0.88) for 10-20 ms
// more lag while it moves far up, and costs nothing near the board (track
// lag 10 ms as before; the field rule has the smoothing off there anyway).
// Higher K buys a little more rest and more lag (0.7: 0.105 mm, +10 ms). A
// moving/at-rest latch with hysteresis was tried and fed on itself the
// same way the old rule did.
#define MAGLOC_SPEED_WINDOW 10
#define MAGLOC_SPEED_JITTER_K 3.0f // (the bench's setting on 2026-09-27, taken as the default: 0.4 until then)

// A magnet is "present" when the strongest smoothed reading passes this. A fix
// is then attempted once three sensors notice it at all - and noticing means
// MAGLOC_FAINT_MT, three times the smoothed noise, not the much higher
// MAGLOC_SEEN_MT of a sensor that sees it plainly. (Both counts are only a
// gate on spending the time. The fit itself always uses EVERY working sensor,
// however little it reads - "almost nothing here" pins a magnet down too - and
// what decides whether the answer is any good is its error bar.) So a weak
// magnet close to two sensors, with its neighbours down at 0.02 mT, locks.
#define MAGLOC_PRESENT_MT 0.02f // the menu's "presence" lever starts here (0.02-0.20); its corroboration - two TMAGs reading plainly - is at the lever too, below MAGLOC_SEEN_MT, so a strong magnet can be met further out where the zeros allow it (2026-09-26; the absent run's toggles say whether they do) (the bench's setting on 2026-09-27, taken as the default: 0.04 until then)
#define MAGLOC_PRESENT_LOW 0.5f // ...and once present it stays so down to this fraction of it (hysteresis) - while a fix has been made in the last MAGLOC_PRESENT_HOLD_MS; with nothing fitting, presence needs the full level again (2026-09-22: a TMAG's 0.028 mT zero error after a boot held presence on at the half level for minutes, and the lattice ran twice a second for nothing)
#define MAGLOC_PRESENT_HOLD_MS 2000
// Presence takes the MMC56x3's word, or two TMAG5273s': no probe can be seen
// plainly by ONE TMAG while the MMC, at the centre, reads nothing (a magnet
// 25 mm over a corner TMAG gives it 0.27 mT and the MMC 0.11); a lone
// TMAG above the level is its zero gone off - after a boot the restored
// zeros are 0.02-0.04 mT stale, and one held presence on for the two
// minutes its absorb clock took, the lattice running the whole time
// (2026-09-22: 470 rejected cold starts before the board went quiet).
// Without an MMC read this frame the old rule stands.
#define MAGLOC_PRESENT_TMAGS 2
// A sensor not read this frame: for this many frames its last smoothed
// reading still says what is there - for presence and the counts, not the
// fit. (The MMC56x3 on its hand-wired bus misses a frame every second or
// so - 2026-09-22: with the probe lying beside the board, seen by the MMC
// alone, its every missed frame ended the track "nothing present" and the
// next frame found it again: 1030 times in 24 minutes, a flicker on the
// LEDs.) The MMC's misses come in bursts of a frame or two (a NACK burst
// of ~12 ms), so five frames; its return after three restarts its history
// (filterFrame's resync) as before.
#define MAGLOC_MISSED_HOLD_FRAMES 5
#define MAGLOC_SEEN_MT 0.04f
// The seen level (and the TMAG's plain level) was set for the 2026-09-18
// magnet of 4232; a weaker magnet is seen plainly by fewer sensors at the
// same level, so the level follows the learned strength (levelScale: the
// ratio to this, 0.25-1), never below twice a frame's noise
// (MAGLOC_SEEN_FLOOR_MT). The presence level does not follow it, nor does
// its corroboration (two TMAGs reading PLAINLY, at 0.04): below 0.04 mT it
// is the zeros' drift (0.015 of uncertainty) that decides, and the absent
// run of the bench protocol sets its floor.
#define MAGLOC_THRESHOLDS_TUNED_AT 4232.0f
#define MAGLOC_SEEN_FLOOR_MT ( 2.0f * MAG_WEIGHT_REFERENCE_MT )
#define MAGLOC_FAINT_MT 0.015f
// A sensor reading under this fraction of its faint level is QUIET: nothing
// there for it, whatever the array says, and its zero follows its reading
// (keepZeros). Between quiet and faint it still counts toward "enough
// sensors notice" and is held while a magnet is present: followed, a 30 mm
// hover's small share at the far sensors was eaten and the fit starved
// (2026-09-23, the bench scene: 7 cold starts "too few noticing" in a minute).
#define MAGLOC_QUIET_FRACTION 0.5f
#define MAGLOC_MIN_SENSORS 3
// Those three levels are a TMAG5273's. A quieter type's are scaled down by
// its noise over the TMAG's (MagArray::noiseMt), so an MMC56x3 at 0.0003
// mT a frame would notice the magnet 40x further down in field - but no
// lower than this fraction of the TMAG levels. At 1/5 the MMC is "present"
// at 0.008 mT (the probe's magnet 100 mm off on its axis), faint at 0.003
// and quiet under 0.00075. Not lower, because presence from one sensor
// alone is decided by its ZERO as much as by its noise: the MMC's zero
// drifts 0.0001 mT a minute on the bench (2026-09-21), the room moves it
// by a few thousandths (a chair, a laptop; the LED chain is 0.0001), and
// at the earlier 1/20 (present at 0.002 mT) a clean zero was "present" with
// nothing there twenty minutes later, the fit cold-starting on it several
// times a second for good. The strongest reading is counted in these
// TMAG-equivalent units too (fix.peakLevel), for presence and for the
// drift rule; fix.peakMt stays the reading itself.
#define MAGLOC_TYPE_SCALE_FLOOR 0.2f

// ...and since a cold start runs in slices (magFitSolveStep: the lattice,
// its refinement, then up to seven seeds), the slices are spread evenly,
// one every MAGLOC_COLD_SLICE_MS, with no pause between one cold start and
// the next: the same work as a 25-38 ms burst every 200 ms, but as a steady
// 40 Hz pattern the supply (and the LCD's backlight, 2026-09-20) does not
// show, and never more than one slice's hold of the loop at a time.
#define MAGLOC_COLD_SLICE_MS 25
// The steady fit load (the tracker page's "fit load": steady, saved): every
// frame with a magnet present runs exactly this many refinement iterations
// - converged or not while tracking, one resumable slice of the cold start
// while hunting - so the V5F's work, and its supply current, is the same
// every 10 ms rather than a burst now and then. Kevin's idea, 2026-09-20.
#define MAGLOC_STEADY_ITERATIONS 4 // the default of "fit iters" (~0.85 ms each on the CH32H417). tools/hostsim/pencil.cpp: 2 a frame tracks exactly as the natural caps do (error, lag and jitter to the last digit); at 1 it loses 0.1 % of frames. More only makes the pulse the supply shows bigger (2026-09-20). (the bench's setting on 2026-09-28 evening, taken as the default: 2 until then)


// Worst residual/signal ratio still called a fix (see magFitSolve). Far from
// the array a real magnet fits to a few percent. Close in it does not: the
// misfit is about 3 x (sensor position error / distance), so a magnet 10 mm
// from two sensors at once turns half a millimetre of table error into 15 %.
// A rough fix that says how rough it is (fix.misfit) beats no fix.
#define MAGLOC_MAX_MISFIT 0.40f

// A fix whose own error bar (all three axes, 1 sigma) is wider than this is not
// offered as a fix at all.
// One acceptance for every fit, near or far, one sensor or nine (2026-09-23;
// the near/far rules, the TMAGs' confirmation and the rough offer before):
// the residuals against each sensor's own expected error (magFitChi: its
// noise as smoothed, its zero's doubt, the model's share of what it reads)
// under this - about 1 is a fit as good as the readings; the bench-like
// scenes' near fixes 0.9-1.3, a 30 mm hover 1.7, the MMC's far fixes 0.8,
// the MMC's 0.02 mT phantom 4.0 (the TMAGs read none of what a probe there
// would give them) - and the bar under MAGLOC_MAX_ERROR_MM (30: the far
// probe at 90 mm has an 18 mm bar; a fix wider than 5, ROUGH_ABOVE_MM, is
// rough). ASSUMPTION: the simulator's world; section 8's captures re-tune it.
#define MAGLOC_FIT_MAX_CHI 6.0f // (the bench's setting on 2026-09-27, taken as the default: 2.0 until then)
// ...and a fix beyond the array's footprint by more than this needs to be
// seen plainly by MAGLOC_OUTSIDE_MIN_SEEN sensors: a planar array cannot tell
// a source past its edge from its mirror (the same readings from a magnet on
// the sensor plane further out), and the wrong basin off the end of the
// board fitted within every sensor's error (chi 0.8-1.9) while two sensors
// saw the probe. Row 1 is 9 mm past the array and four sensors see it.
#define MAGLOC_OUTSIDE_MM 10.0f
#define MAGLOC_OUTSIDE_MIN_SEEN 3
// A cold start (the lattice) rejected twice running is not tried again for
// this long, in either fit load: on a phantom it would run every slice. Once
// is not held against it: a probe that has just arrived (or jumped a row) is
// fitted on fields the smoothing is still settling, and that fit fails the
// chi; the next slice's does not.
#define MAGLOC_COLD_RETRY_MS 400
#define MAGLOC_MAX_ERROR_MM 30.0f
// With the tracker OFF there is no track to start the next fit from: a
// refused fit keeps its own answer as the start for this many frames running
// before the cold start (the tracker's coast does this with it on). A fast
// hand's frame (the sensors read round-robin over it) fails the chi; the cold
// start's slices and its retry hold-off were then a half-second gap on the
// LEDs on every fast approach (2026-09-26, the bench with the tracker off).
#define MAGLOC_RAW_WARM_FRAMES 10

// The probe's magnet strength, mT*mm^3, that the fit starts out holding
// (held, the height noise about halves - magFitSolveKnownStrength - and far
// out it is what makes a fix at all). It is LEARNED from the first near
// fixes on (MAGLOC_STRENGTH_*, MagLocator::measureStrength) and held at
// what was learned, across boots too (settings: strength=): a weaker
// magnet is not a setting to get right. (2026-09-22: "a bit weaker than
// earlier" - held at 4232, a magnet of 2100 fitted 5 mm too high and 20-40
// mm off along the board, "valid" at a 24 % misfit and a 1.6 mm bar, and
// the row calibration never took a tap: the fitted height failed its
// on-the-board test. Held at 2900 the taps took, 3 mm high and up to 6 mm
// off.) 0 = never hold it (the console's K).
#define MAGLOC_MAGNET_STRENGTH 3480.0f // the probe's, MEASURED on the bench 2026-09-22 23:50 (263 frames at 6 places); the 2026-09-18 probe's was 4232 (magcal 4213 +/- 4.5 %). A wiped board starts here; the learning takes over from the first near fixes (the bench's magnet as learned on 2026-09-27, taken as the default: 1839 until then)
// Why it is held from boot rather than learned (2026-09-21): far from the
// array only the MMC56x3 reads the magnet plainly, and its three numbers
// are fitted EXACTLY by a weak magnet just over it or the real one far up -
// the free fit cannot tell, and settled on "row 13, z 6 mm, strength 103"
// for the probe hovering an inch over the board. With the strength held
// the far answer is the only one. The direction is the other thing three
// numbers cannot pin: the last good fix's pole is a soft prior on it
// (magFitRefineKnownStrength), pulling with this much weighted mT per unit
// of direction - about a TMAG's smoothed noise, so it only ever decides
// what the readings leave open. (The sim: a probe 90 mm up comes back
// within 3.4 mm with the prior right, 25 mm off with it 40 degrees wrong.)
#define MAGLOC_AXIS_PRIOR_MT 0.02f // ...only where the direction is open: by the fit's own bar on the AXIS since 2026-09-28 evening (MAGLOC_PRIOR_FROM_RAD below; that afternoon by the position bar, MAGLOC_PRIOR_FROM_MM, which the MMC's far share still goes by)
#define MAGLOC_PRIOR_FROM_MM 8.0f
// ...that was the position bar's say (2026-09-28 afternoon); since the evening
// the prior is ramped by the fit's own bar on the AXIS (MagFitResult::
// axisSigma, smoothed over MAGLOC_FAR_BAR_TAU_S): none under this, all of it
// from twice it. Near the board the fit reports 4-8 degrees and pins the
// axis itself; 40 mm up it reports 18-23 (the sim and the bench alike) and
// the bench's fit there sat in the position-against-tilt valley - x 6 mm and
// 60 degrees for a centred probe at 10, the aim cursor 100 mm out - which is
// what the prior (the last NEAR fix's pole: the lean the hand lifted with)
// breaks. The afternoon's sim case that switched it off under an 8 mm bar
// had no near fix first, so its prior pulled toward the compiled-in
// reference axis at 41 degrees - not a bent axis, a stale target.
#define MAGLOC_PRIOR_FROM_RAD 0.17f // radians (10 degrees)
// The boot zero check (adoptBootZero): the saved zero is put back and then
// judged against the first MAG_BASELINE_FRAMES of readings - their difference
// adopted as the zero when no dipole explains it (drift since it was saved),
// kept when one does (the probe lying near the board). OFF since 2026-09-28
// evening (Kevin: "remove the boot calibration so you don't need me to move
// the probe"): a probe 40 mm up reads 0.02-0.04 mT, which is not a dipole
// the fit can name, and every reflash with it in place had the check eat it.
// The saved zero is put back and kept; drift is the offset filter's
// (keepZeros), the audit's and z's. 1 puts the check back; the host sim
// builds with it on (its Makefile), so boot.txt and bootprobe.txt keep it honest.
#ifndef MAGLOC_BOOT_ZERO_CHECK
#define MAGLOC_BOOT_ZERO_CHECK 0
#endif
#define MAGLOC_FAR_BAR_TAU_S 0.25f // s: the fixes' bar is smoothed over this before it ramps the prior and the MMC's far share (a fix's own bar jitters +/-40 % 55 mm up, and the ramp flapped on it)
// ...and how many iterations that refinement may spend a frame (the steady
// load's "fit iters" when that is on): from the free fit's answer it needs
// two or three, and a frame's fit must not run long (the supply shows it).
#define MAGLOC_REFINE_ITERATIONS 8

// Learning the strength (the comment at MAGLOC_MAGNET_STRENGTH): a frame
// measures it when this many sensors read the magnet PLAINLY (the readings
// pin the strength themselves; the free fit runs with no hint) and the free
// fit is sharp (misfit, error bar). The answers go into a ring; the held
// strength is TAKEN from the ring's median once the ring is MIN_SAMPLES
// deep and its middle half agrees to MAX_SPREAD - once, and then held
// still: a value that followed the median drifted with each place's own
// few percent of calibration bias, and fixes taken a few seconds apart
// disagreed (the tip solve from three angles went from 0.03 to 0.18 mm rms
// in the sim). It is taken again when the median is JUMP away (another
// magnet: at once), after `k`, or when the median has sat RETAKE_STEP
// away for RETAKE_FRAMES measuring frames (a slow, real change, thirty
// seconds of near fixes). The settings keep it once it has stood SAVE_STEP
// from what they hold for SAVE_MS (one write a minute at most).
#define MAGLOC_LEARN_MAX_MISFIT 0.10f
#define MAGLOC_LEARN_MIN_SENSORS 4 // (5 until 2026-09-23: with the 1839 magnet five sensors read it plainly only over the middle rows, and the ring starved)
#define MAGLOC_STRENGTH_MAX_ERROR_MM 3.0f
#define MAGLOC_STRENGTH_RING 32
#define MAGLOC_STRENGTH_MIN_SAMPLES 16
#define MAGLOC_STRENGTH_MAX_SPREAD 0.10f
#define MAGLOC_STRENGTH_JUMP 0.20f
#define MAGLOC_STRENGTH_RETAKE_STEP 0.05f
#define MAGLOC_STRENGTH_RETAKE_FRAMES 3000
// ...and a measurement is the median over PLACES: each full, agreeing ring
// whose mean position is PLACE_APART from every place remembered adds a
// place (the last PLACES kept); the strength is taken from MIN_PLACES of
// them. One place's answer carries that place's own calibration bias (the
// sim's 1-2 %, the bench's ±4.5 %), and frozen in it put the far fix and
// the tip solve off; over places it averages out. (A jump - another
// magnet - is taken from the ring alone, wherever the probe is.)
#define MAGLOC_STRENGTH_PLACES 8
#define MAGLOC_STRENGTH_MIN_PLACES 4
#define MAGLOC_STRENGTH_PLACE_APART_MM 5.0f
#define MAGLOC_STRENGTH_SAVE_STEP 0.05f
#define MAGLOC_STRENGTH_SAVE_MS 60000
// Settling a provisional zero (MagArray::zeroProvisional): after this many
// good fixes running, and only when the magnet's field at the sensor is at
// least this much (else nothing can be told about its zero).
#define MAGLOC_SETTLE_GOOD_FIXES 20
#define MAGLOC_SETTLE_MIN_MT 0.02f

// Where the probe's POINT is, and how the magnet sits on the probe.
//
// The fit gives the magnet's pole direction. The magnet is mounted at some
// ANGLE to the shaft: 0 = magnetised along it (a coin on the end of the rod, a
// ring slid over it), 90 = across it (a disc lying flat on its side). A single
// dipole's field is symmetric about its pole, so what the fit can see of the
// probe's direction is only what the pole shows: with the magnet along the
// shaft that is everything; with it across, the shaft can be anything
// perpendicular to the pole, and the one taken is the most nearly vertical -
// right when the probe is held upright, off by (offset) x sin(lean) for a
// lean sideways to the pole, which it cannot see. In between, in between.
// The shaft is turned to point up (a probe is not held upside down), and the
// point is MAGLOC_TIP_OFFSET_MM down it from the magnet's centre. Both can be
// typed: t<mm> and T<degrees>.
#define MAGLOC_TIP_OFFSET_MM 6.7f // (the bench's setting on 2026-09-28 evening, taken as the default: 0.0 until then)
#define MAGLOC_MAGNET_ANGLE_DEG 0.0f

// Where the probe POINTS: carry the point on down the shaft to the breadboard's
// surface, this high above the sensors, and no further (the tracker's cursor,
// MagTracker.h). With the point resting on the board that is the point itself;
// held above it, it is the hole the probe is aimed at rather than the one
// under it. 0 = not known: the sensor plane is used. The row calibration
// (`c`) sets it from the height the point rests at; `S` types it. On the
// bench the breadboard sits 17.5 mm over the sensors; on V6 the surface is
// 7.1 mm above the base PCB (plus wherever the sensors sit below that).
#define MAGLOC_BOARD_Z_MM 9.7f // (the bench's setting on 2026-09-27, taken as the default: 17.5 until then) (the bench's setting on 2026-09-28 evening, taken as the default: 14.5 until then)

// A fix goes to the tracker as a ROUGH one ("somewhere about here", the wide
// glow on the LEDs, no row counted) rather than a proper one when its error
// bar is wider than this - a probe far up or off the edge. Rough is the
// fit's own bar, not a class: a fit the acceptance refuses is nothing (the
// rough offer of a refused fit went 2026-09-23, Phase 3 step 4b).
#define MAGLOC_ROUGH_ABOVE_MM 5.0f

// The baseline check. A baseline taken with the probe lying on the board
// subtracts the magnet's own field from every reading after it, and nothing
// fits until `z` is pressed with the probe away - which after a reflash is
// easy to forget. Sensor offsets (up to a mT, but random from sensor to
// sensor) and the earth's field (the same at every sensor) do not look like
// a dipole; a magnet does. So each new baseline, less its mean across the
// sensors, is fitted as a dipole, and if one explains it (misfit under the
// limit) with at least this much strength, the baseline is called polluted:
// a warning is printed and shown, and it is retaken a few times.
#define MAGLOC_BASELINE_MAGNET_MISFIT 0.3f
#define MAGLOC_BASELINE_MAGNET_STRENGTH 300.0f // mT*mm^3 (a probe magnet is thousands)
#define MAGLOC_BASELINE_RETAKES 3
#define MAGLOC_BASELINE_RETAKE_MS 2000

// While nothing is present the zero the drift has settled on IS the zero:
// a sensor's saved zero follows it whenever the two differ by this much
// (MagArray::settleDriftedZeros), so a record that came back wrong at boot
// (2026-09-21: the MMC's, saved under an older table row) is right for the
// next boot too, and the settings are not rewritten for every thousandth.
// The ZERO AUDIT (ZeroAudit.h): while the probe is near and the fit is good
// (misfit under MAGLOC_AUDIT_MAX_MISFIT, at least MAGLOC_AUDIT_MIN_SEEN
// sensors seeing it plainly, the bar under MAGLOC_AUDIT_MAX_ERROR_MM), once
// every MAGLOC_AUDIT_PERIOD_MS the fit is redone without one sensor (round
// robin, one sensor a period) and that sensor's reading is set against the prediction. With
// MAGLOC_AUDIT_MIN_SAMPLES of those over enough spread of position
// (MAGLOC_AUDIT_MIN_SPREAD: the prediction's rms spread over its mean) its
// zero and gain are solved together; a zero past MAGLOC_AUDIT_APPLY_MT (and
// under MAGLOC_AUDIT_MAX_MT, a sanity bound) is taken out of the baseline
// and the settings keep the corrected zero. This and `z` are the only
// things that write a zero to flash: what a far-only absorb does to the
// live baseline stays live (2026-09-21 night, after the settle-to-flash of
// the evening made a real hover's absorption permanent).
#define MAGLOC_AUDIT_PERIOD_MS 100 // one sensor a period, round robin: each about once a second
#define MAGLOC_AUDIT_MAX_MISFIT 0.10f
#define MAGLOC_AUDIT_MIN_SEEN 5 // (6 until 2026-09-23: the 1839 magnet is seen plainly by six sensors from few places)
#define MAGLOC_AUDIT_MAX_ERROR_MM 3.0f
#define MAGLOC_AUDIT_MIN_SAMPLES 40
#define MAGLOC_AUDIT_MAX_SAMPLES 400 // a window that never crosses the threshold starts again after this many (an hour's drift is not a week's)
#define MAGLOC_AUDIT_MIN_SPREAD 0.3f
#define MAGLOC_AUDIT_APPLY_MT 0.002f         // ...or 0.6 of the sensor's own noise, whichever is more (a TMAG: 0.007; the MMC this floor - 0.004 until 2026-09-24, thirteen times its noise, left a 0.0035 mT error the follow had not finished with; two windows agreeing guard against noise now)
#define MAGLOC_AUDIT_APPLY_NOISE 0.6f
// A gain is corrected past this (the datasheet's typical spread: under it a
// 40-sample solve is noise - in a clean world the solve scatters +-3 % and
// at 1 % the audit "corrected" sixteen times in ten minutes, a flash write
// each, 2026-09-23), past three of the solve's own sigma, and only when two
// windows running agree within MAGLOC_AUDIT_GAIN_AGREE (the solve's sigma is
// not honest for the bench's systematic errors: with a misplaced sensor in
// the fits one Z gain swung 0.68, 1.57, 0.76 window after window).
// ...and a gain only past 20 % (the datasheet's MAXIMUM spread: a real bad
// sensor, the 09-21 TMAG at 64 %), because a leave-one-out audit cannot tell
// a sensor's gain from another sensor's place: with one sensor 1 mm off in
// the bench-like world the solve put a consistent 16 % on a true sensor's Y
// (2026-09-23). Under 20 % the gain is reported (:audit) for magcal.
#define MAGLOC_AUDIT_APPLY_GAIN 0.20f
#define MAGLOC_AUDIT_GAIN_AGREE 0.03f
#define MAGLOC_AUDIT_GAIN_MIN_MT 0.02f      // a gain is judged only on an axis whose predicted signal is at least this (rms, twice a TMAG's noise): at the noise floor a Z "gain" of 0.65 was read off nothing
#define MAGLOC_AUDIT_ZERO_AGREE 0.005f       // ...and a zero likewise: two windows running within this (in a clean world one window said +0.012 mT, the next -0.014: a walk, each a flash write)
#define MAGLOC_AUDIT_MAX_GAIN_TRIM 2.0f      // a gain further from 1 than this (either way) is a row that wants magcal, not a trim; the trim itself never goes past it in all
#define MAGLOC_AUDIT_MAX_GAIN_ERROR 0.15f    // a sensor whose gain solves further from 1 than this...
#define MAGLOC_AUDIT_MAX_UNEXPLAINED 0.15f   // ...or whose residual after the solve is more than this of its field, is left out of the audit's fits
#define MAGLOC_AUDIT_MAX_MT 0.3f
#define MAGLOC_AUDIT_ITERATIONS 6
#define MAGLOC_ZERO_SETTLE_MT 0.01f
// ...and not more often than this: a settle is a settings write (a page
// erase, the LED chain blinked off for it), and a zero that only matters at
// the next boot can wait. The first after a boot does not wait.
#define MAGLOC_ZERO_SETTLE_MIN_MS 600000
#define MAGLOC_SEEN_ABSORB_LEVEL ( 2.0f * MAGLOC_SEEN_MT ) // "plainly": twice the seen level (the MMC: 0.016 mT, a probe within 80 mm, which fits with a 10 mm bar)

// A reference fix: the probe resting in row 35, hole 3, at 41 degrees, as
// fitted on 2026-09-18 with a zero taken while it was away (medians of two
// minutes of fixes, jitter 0.1 mm). Y1 takes this magnet out of a fresh
// zero taken with the probe still there.
#define MAGLOC_REFERENCE_X 1.14f
#define MAGLOC_REFERENCE_Y 9.35f
#define MAGLOC_REFERENCE_Z 10.82f
#define MAGLOC_REFERENCE_AX -0.257f
#define MAGLOC_REFERENCE_AY 0.606f
#define MAGLOC_REFERENCE_AZ -0.753f
#define MAGLOC_REFERENCE_STRENGTH 4232.0f

// Display smoothing: fraction of each new fix blended in (1 = none).
#define MAGLOC_SMOOTHING 0.5f

struct MagProbeFix {
    bool present;    // something magnetic is near the array
    bool valid;      // ...and a dipole explains it; the fields below are good
    bool rough;      // ...but only roughly: the error bar is wider than MAGLOC_ROUGH_ABOVE_MM (the tracker treats it as a rough fix)
    Vec3 magnet;     // magnet centre, mm, board frame (smoothed for display, MAGLOC_SMOOTHING)
    Vec3 tip;        // the probe's point, mm (smoothed the same way)
    Vec3 rawMagnet;  // magnet centre exactly as this frame's fit gave it - the one `sigma` is the error bar of
    Vec3 rawTip;     // ...and the point from that
    Vec3 pointer;    // where the shaft, carried on from the point, meets the board's surface (smoothed)
    Vec3 rawPointer; // ...from this frame's fit
    Vec3 axis;       // unit vector along the magnet, S -> N
    Vec3 shaft;      // unit vector up the probe's shaft, as far as the pole shows it (see MAGLOC_MAGNET_ANGLE_DEG)
    float tiltDeg;   // angle between the shaft and the board normal, 0..90
    float strength;  // |moment|, mT*mm^3 (constant for one magnet - a sanity check)
    float residual;  // RMS misfit, mT
    float misfit;    // residual / signal: 0.03 is a clean fix, 0.3 a rough one
    float chi;       // the rms of (reading - model) over each sensor's own expected error (its noise as smoothed, its zero's doubt, the model's share): 1 is a fit as good as the readings, 2 is not
    bool rejectedOutside; // the last fit was refused for being beyond the array's footprint on too few plain sensors (the mirror basin), not by the chi or the bar
    Vec3 sigma;      // 1-sigma error bar on the magnet position, mm, per axis (from the fit itself)
    float errorXyMm; // ...across the board: sqrt(sx^2 + sy^2)
    float errorMm;   // ...and in all three axes together
    float axisSigmaRad; // ...and on the axis's direction, radians (the known-strength fit's; 0 = the sim's probe)
    float peakMt;    // strongest (smoothed) reading
    float peakLevel; // ...in TMAG-equivalent units (a quiet type's reading scaled up by its noise ratio): what presence is judged by
    int seenBy;      // sensors reading above MAGLOC_SEEN_MT
    int faintBy;     // sensors reading between MAGLOC_FAINT_MT and MAGLOC_SEEN_MT
    uint32_t fitUs;  // how long the fit took
    uint32_t count;  // fixes since boot
};

class MagLocator : public Service {
  public:
    static MagLocator& getInstance( );

    MagLocator( const MagLocator& ) = delete;
    MagLocator& operator=( const MagLocator& ) = delete;

    void begin( );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "MagLocator"; }
    ServicePriority getPriority( ) const override { return ServicePriority::HIGH; }
    uint32_t periodUs( ) const override { return 2000; }

    MagProbeFix fix = { };
    bool streaming = false;

    // The probe over time: the fixes above filtered, gated and carried
    // through gaps, and the cursor on the board's surface (see MagTracker.h).
    // Console: u = cursor under the tip / where it points, S = the surface
    // height (S17<Enter>), g = tracker on/off (off = raw fixes, for comparing).
    MagTrack track = { };
    ZeroAudit audit;           // each sensor's zero against the fit of the others (ZeroAudit.h, `:audit`)
    void printAudit( Stream* out ) const;

    // The baseline check (see MAGLOC_BASELINE_*): true while the baseline in
    // use looks like it has a magnet in it.
    bool baselinePolluted = false;
    float baselineMagnetMisfit = 0.0f, baselineMagnetStrength = 0.0f;

    void printFix( Stream* out ) const;

    // A simulated probe (the :probe verb, the host simulator): the tracker
    // is fed this magnet position and shaft instead of a fit - the array and
    // its sampler keep running, the fit is skipped - and `fix` is filled as
    // a fit would fill it, so everything downstream (rows, LEDs, paint, the
    // scene with its SIM mark) sees a probe that is exactly where it is said
    // to be. ms 0 = until simProbeOff().
    struct ProbeSim {
        bool on;
        uint32_t untilMs; // 0 = no end
        Vec3 position;    // the magnet's centre
        Vec3 shaft;       // unit, up the probe
        float sigmaMm;
        bool rough;
    };
    ProbeSim sim = { };
    bool fitHeld = false;   // :load fit off - the frames flow, nothing is fitted (the fit is the V5F's heaviest work)
    bool steadyFit = false;                          // the fit's work the same every frame instead of bursts (the tracker page's "fit load")
    float steadyIterations = MAGLOC_STEADY_ITERATIONS; // ...and how much: refinement iterations a frame ("fit iters", saved)
    uint32_t coldWhyAbsent = 0, coldWhyFew = 0, coldWhyCoasted = 0, coldWhyRejected = 0; // what ended the track before each cold start: nothing present, too few noticing, the fit failing until the track coasted out; and cold starts rejected (chi, the bar, beyond the array)
    uint32_t staleFrames[ MAGFIT_MAX_SENSORS ] = { 0 }; // frames in which a sensor was not read (the MMC56x3's 150 Hz against 100 Hz frames)
    uint32_t presenceToggles = 0; // times presence went on or off: the `d` stream prints no absent frame, so a toggle shows there only as a gap (the bench protocol, 2026-09-23)
    uint32_t refineRejected = 0;  // frames in which only the held strength's refinement failed the acceptance and the free fit went on (:load)
    uint32_t missFrames = 0;      // frames that gave no fix while the track was alive and were ridden through (the fit starts again from the track): each a near miss of a cold start
    float maxPeakLevel = 0.0f;    // the strongest smoothed reading (TMAG terms) since :load last printed it: the absent run's presence floor
    uint32_t presenceHeldFrames = 0; // frames in which presence rested on a sensor's last reading, the sensor not read that frame (MAGLOC_MISSED_HOLD_FRAMES)
    uint32_t coldStarts = 0, coldStartUs = 0, coldSliceMaxUs = 0; // cold starts since boot; the last one's cost in all, and its longest slice (one per frame)
    void simProbeSet( Vec3 position, Vec3 shaft, float sigmaMm, bool rough, uint32_t ms );
    void simProbeOff( );
    bool simProbeActive( ) const { return sim.on; }

    // The magnet strength the fit is held to (0 = free: K), learned from
    // the near fixes (MAGLOC_STRENGTH_*; the comment at MAGLOC_MAGNET_STRENGTH).
    float knownStrength = MAGLOC_MAGNET_STRENGTH;
    float strengthMedian = 0.0f;   // the ring's median, the last time it was deep enough (0 = not yet)
    int strengthPlaces( ) const { return strengthPlaceCount; }
    float strengthSpread = 0.0f;   // ...and its middle half's spread, relative
    float strengthSaved = MAGLOC_MAGNET_STRENGTH; // what the settings hold (they write it when this changes)
    uint32_t strengthMeasured = 0; // frames that measured it
    uint32_t strengthJumps = 0;    // times another magnet was taken at once (MAGLOC_STRENGTH_JUMP)
    void startLearningStrength( ); // k: hold it, and measure it again from scratch
    void holdStrength( float strength ); // :strength <n>: a known magnet, held as measured
    void forgetStrength( );        // K: never hold it (fit it freely)
    void restoreStrength( float strength, bool measured ); // the settings' record at boot: `strength=<n> measured` is trusted as learned; a bare record (a default that was saved) is held but not trusted, so the first measurement over enough places takes it as on a fresh board
    bool strengthIsMeasured = false;                 // the held strength came from the ring (this boot or a measured record), not from a default
    bool strengthSavedMeasured = false;              // ...and so did the value the settings hold (strengthSaved): what they write as ` measured`, else ` unconfirmed` (the seal follows the saved value, not the held one: a minute apart, 2026-09-23)

    // How far the magnet's centre is up the shaft from the probe's point, and
    // the magnet's angle to the shaft.
    float tipOffsetMm = MAGLOC_TIP_OFFSET_MM;
    TipModel tipModel;       // the point for a leaning shaft, learned by the lean calibration (TipModel.h); invalid = the scalar tip above
    float tipModelD = 0.0f;  // the tip the model was learned with: "tip" edited by hand away from it drops the model (a hand-set tip is the plain one)
    Vec3 pointOf( Vec3 magnet, Vec3 shaft ) const { return tipModelPoint( &tipModel, tipOffsetMm, magnet, shaft ); }
    void learnTipModel( const TipModel& m, float tipMm ); // the calibration's result, in use and remembered
    void forgetTipModel( );
    float magnetAngleDeg = MAGLOC_MAGNET_ANGLE_DEG;
    float boardZ = MAGLOC_BOARD_Z_MM; // the breadboard's surface at the taps' centre, mm above the sensors (the menu's "surface", saved; the row calibration sets it)...
    SurfaceMap surfaceMap;            // ...and how much higher or lower it runs elsewhere (SurfaceMap.h; the row counter fits it to its anchors, flat until then)
    float surfaceAt( float x, float y ) const { return boardZ + surfaceMapOffset( &surfaceMap, x, y ); } // the surface under (x, y): the height, the pointer's plane and the touch use it
    uint32_t surfaceLearned = 0; // times the surface came down to the floor
    float presentMt = MAGLOC_PRESENT_MT; // the strongest smoothed reading that counts as a magnet (menu: presence)
    float fitMaxChi = MAGLOC_FIT_MAX_CHI; // the fit's acceptance: residuals over each sensor's own expected error (menu: fit chi - loosened at the bench when its errors are not the simulator's)
    float levelScale( ) const;           // the seen level's scale for the held strength (MAGLOC_THRESHOLDS_TUNED_AT)
    float seenLevelMt( ) const;          // ...and the level itself
    // The last good fix (a sharp one, misfit under MAGLOC_LEARN_MAX_MISFIT):
    // kept in the settings, so that after a reboot with the probe lying where
    // it was, `Y` can take that magnet back out of the baseline the boot
    // zeroed it into (unpolluteBaseline). Starts as the fix measured on
    // 2026-09-18 with the probe resting in row 35, hole 3, at 41 degrees.
    Vec3 lastGoodPosition = { MAGLOC_REFERENCE_X, MAGLOC_REFERENCE_Y, MAGLOC_REFERENCE_Z };
    Vec3 lastGoodAxis = { MAGLOC_REFERENCE_AX, MAGLOC_REFERENCE_AY, MAGLOC_REFERENCE_AZ };
    float lastGoodStrength = MAGLOC_REFERENCE_STRENGTH;
    bool haveLastGood = true;
    float farBar = 2.0f * MAGLOC_PRIOR_FROM_MM; // the accepted fixes' 3D bar, smoothed (MAGLOC_FAR_BAR_TAU_S): what the MMC's far share is ramped by
    float farAxis = 2.0f * MAGLOC_PRIOR_FROM_RAD; // ...and their AXIS bar, smoothed the same: what the axis prior is ramped by
    bool farBarLive = false;                  // ...seeded by the first fix after none
    bool baselineCorrected = false; // Y has been applied to the zero in use (a new zero clears it)
    void unpolluteBaseline( Stream* out );
    int provisionalGoodFixes = 0; // good fixes running, toward settling a provisional zero

    // Bench tool for getting MagArrayConfig.h right. With a magnet held a
    // centimetre or two over the array (off-centre is best), fit it under every
    // combination of: the two rows swapped or not, each row's order along x
    // turned round or not, each row's package rotation (0/90/180/270), and top
    // side or underside. A wrong combination cannot be
    // explained by any dipole, so the misfit singles out the right one. Try it
    // at two or three magnet positions; the true combination wins every time.
    //
    // What it cannot decide: every answer appears four times - itself, its
    // mirror images (which put the parts on the other side of the board; the
    // mirror image of a magnet and sensors is magnetically just as valid), and
    // the same array with the frame turned half way round. Knowing which side
    // the parts are on and how one package is turned picks the line.
    void printOrientationCheck( Stream* out ) const;

    // The field smoothing as it stands, for the reports and the pencil bench
    // (tools/hostsim/pencil.cpp): the track's speed over the last
    // MAGLOC_SPEED_WINDOW frames, the alpha last applied (1 = none), and the
    // jitter lever.
    float smoothSpeed = 0.0f;
    float smoothAlpha = 1.0f;
    float speedJitterK = MAGLOC_SPEED_JITTER_K;

  private:
    MagLocator( ) = default;

    MagFitResult result = { };
    Vec3 smooth[ MAGFIT_MAX_SENSORS ] = { };      // the fields the fit sees
    MagOffsetFilter offsets[ MAGFIT_MAX_SENSORS ] = { }; // each sensor's zero as a state with a doubt (MagOffsetFilter.h): the live baseline is set from it every frame (keepZeros)
    Vec3 offsetWritten[ MAGFIT_MAX_SENSORS ] = { };      // ...as last written, so a baseline moved by anyone else (z, Y, the audit, a restore) restarts the filter from it
    bool offsetsBegun = false;
    float zeroDoubt[ MAGFIT_MAX_SENSORS ] = { 0 }; // how far each zero may be off, as the acceptance doubts it (mT): the boot uncertainty, decaying while followed, growing at the drift rate while held (MagArrayConfig.h)
    int freeWarmRun = 0;           // frames running in which only the held-strength refinement failed and the free fit was the next start (bounded: MAGLOC_STRENGTH_MIN_SAMPLES)
    uint32_t lastPresentMs = 0; // when a magnet was last present (0 = never): the follow's hold-off, MAG_OFFSET_HOLDOFF_S
    uint32_t unexplainedSinceMs[ MAGFIT_MAX_SENSORS ] = { 0 }; // since when this sensor has read the magnet's level with no fix explaining it (0 = it does not): the one clock, MAG_OFFSET_HOLD_S
    Vec3 recent[ MAGFIT_MAX_SENSORS ][ 2 ] = { }; // each sensor's last two frames, for the glitch filter
    int missedFrames[ MAGFIT_MAX_SENSORS ] = { }; // frames in a row each sensor was not read
    uint32_t lastFrame = 0;
    bool haveSmoothed = false;
    uint32_t nextColdStartMs = 0;
    uint32_t streamTick = 0;
    float strengthRing[ MAGLOC_STRENGTH_RING ]; // the free fit's strengths, the last frames that measured it
    int strengthRingAt = 0, strengthRingCount = 0;
    bool strengthTaken = false;    // the held strength has been measured (or restored): the ring only watches for a change now
    uint32_t strengthAwayFrames = 0; // measuring frames in a row with the median MAGLOC_STRENGTH_RETAKE_STEP from the held value
    Vec3 strengthRingPlace[ MAGLOC_STRENGTH_RING ]; // where each ring entry was measured
    Vec3 strengthPlace[ MAGLOC_STRENGTH_PLACES ];   // the places measured at (MAGLOC_STRENGTH_PLACE_APART_MM apart)...
    float strengthPlaceValue[ MAGLOC_STRENGTH_PLACES ]; // ...and the ring's median at each
    int strengthPlaceCount = 0, strengthPlaceAt = 0;
    float strengthPlaceMedian = 0.0f; // the median over the places (0 = fewer than MIN_PLACES yet)
    void measureStrength( float strength, Vec3 at );
    uint32_t strengthAwaySinceMs = 0; // since when the held strength has been MAGLOC_STRENGTH_SAVE_STEP from the settings' (0 = it is not)
    void keepStrengthRecord( uint32_t nowMs );

    uint32_t lastTrackUs = 0;
    uint32_t checkedBaseline = 0;          // magArray.baselineCount last checked
    // The zero put back at boot is checked against the live readings once
    // MAG_BASELINE_FRAMES of them are in: their difference, if no dipole
    // explains it, is drift since the zero was saved and the readings are
    // adopted as the zero; if a dipole does, a magnet was near at boot and
    // the saved zero is kept (Kevin's bench, 2026-09-24: 0.05 mT read with
    // the probe nowhere near, from a zero saved days before).
    bool bootAdopting = false;
    int bootAdoptFrames = 0;
    Vec3 bootSum[ MAGFIT_MAX_SENSORS ] = { };
    int bootFrames[ MAGFIT_MAX_SENSORS ] = { 0 };
    void adoptBootZero( );
    bool magnetExplains( const Vec3* fields, const bool* use, int count, MagFitResult* out ) const; // the pollution test: the fields less their mean fit a dipole of a magnet's strength over the board
    Vec3 speedRing[ MAGLOC_SPEED_WINDOW ]; // the track's position, the last MAGLOC_SPEED_WINDOW frames
    int speedRingCount = 0;
    int baselineRetakes = 0;
    uint32_t retakeBaselineAtMs = 0;
    void checkBaseline( );
    // One frame's scratch, handed from stage to stage (fitFrame).
    struct FrameScratch {
        Vec3 filtered[ MAGFIT_MAX_SENSORS ]; // the readings through the glitch filter
        bool resync[ MAGFIT_MAX_SENSORS ];   // a sensor back after missing frames: its history starts again
        bool use[ MAGFIT_MAX_SENSORS ];      // may vote in the fit
        float alpha = 1.0f;                  // the smoothing this frame
        float weights[ MAGFIT_MAX_SENSORS ]; // each sensor's weight in the fit this frame
        float far;                           // how open the direction is, 0..1, by the last accepted fix's 3D bar (MAGLOC_PRIOR_FROM_MM): the axis prior's weight, and the MMC's share in its far mode
        float farAxis; // ...and the axis prior's, 0..1, by the fixes' smoothed AXIS bar (MAGLOC_PRIOR_FROM_RAD)
        float held = 0.0f;                   // the strength held (0 = free)
        int plain = 0;                       // sensors reading plainly (above their plain level)
        bool hintFree = false;               // the free fit ran with no strength hint: the readings pinned it (a measurement)
        bool enough = false;                 // enough sensors notice it to try a fit
        bool good = false;                   // the fit's verdict
        bool outside = false;                // the fit's answer is beyond the array's footprint (MAGLOC_OUTSIDE_MM)
        bool explained = false;              // an accepted fix that two or more sensors make (or a rough offer a dipole explains): the offsets hold this frame
        bool farOnlyFix = false;             // an accepted fix that ONE sensor makes: real or a zero error's tail, held MAG_OFFSET_FAR_HOLD_S then followed
        bool quiet[ MAGFIT_MAX_SENSORS ];    // this sensor reads under MAGLOC_QUIET_FRACTION of its faint level: nothing there for it, whatever the array says
    };
    bool filterFrame( FrameScratch& f );
    void assessFrame( FrameScratch& f );
    void keepZeros( const FrameScratch& f );
    ServiceStatus runFit( MagTrackInput* in, FrameScratch& f );
    ServiceStatus publishFix( MagTrackInput* in, const FrameScratch& f );
    ServiceStatus fitFrame( MagTrackInput* in );
    ServiceStatus simFrame( MagTrackInput* in );
    uint32_t lastZeroSettleMs = 0; // when the saved zeros last followed the drifted ones (MAGLOC_ZERO_SETTLE_MIN_MS) - unused since the audit took over the writing
    uint32_t lastAuditMs = 0;      // the zero audit's last sample
    uint32_t lastFixMs = 0; // the last accepted fix: the presence hysteresis (MAGLOC_PRESENT_HOLD_MS)
    int auditNext = 0;             // the sensor it looks at next (round robin)
    void auditZeros( const FrameScratch& f );
    float chiSmoothed( const MagFitResult& r, const bool* use, float alpha ) const; // magFitChi with each sensor's error as it stands this frame: its noise as the smoothing left it (alpha), its zero's doubt (the offset filter's, never under magSensorTypeZeroMt), the model's share
    uint32_t coldRetryUntilMs = 0; // a cold start rejected twice running: not again before this (MAGLOC_COLD_RETRY_MS)
    int coldRejectedRun = 0;       // cold starts rejected in a row
    int missRun = 0;               // frames in a row the fit gave no fix (the track carrying on)

    Vec3 pointerOf( Vec3 tip, Vec3 shaft ) const;
    void printFixCsv( Stream* out ) const;
};

extern MagLocator& magLocator;

#endif // MAGLOCATOR_H
