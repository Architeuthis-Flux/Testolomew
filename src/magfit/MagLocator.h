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
#include "ZeroAudit.h"
#include "Vec3.h"

// Weak magnets. One frame carries about 0.011 mT of noise per axis (measured), and a small
// magnet two sensor pitches away is not much more than that, so the fields
// are smoothed before anything looks at them - heavily when the strongest
// reading is weak, not at all once it passes MAGLOC_FAST_MT, so a strong magnet
// still tracks without lag. Smoothed noise is about 0.005 mT.
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
#define MAGLOC_SPEED_JITTER_K 0.4f

// A magnet is "present" when the strongest smoothed reading passes this. A fix
// is then attempted once three sensors notice it at all - and noticing means
// MAGLOC_FAINT_MT, three times the smoothed noise, not the much higher
// MAGLOC_SEEN_MT of a sensor that sees it plainly. (Both counts are only a
// gate on spending the time. The fit itself always uses EVERY working sensor,
// however little it reads - "almost nothing here" pins a magnet down too - and
// what decides whether the answer is any good is its error bar.) So a weak
// magnet close to two sensors, with its neighbours down at 0.02 mT, locks.
#define MAGLOC_PRESENT_MT 0.04f // the menu's "presence" lever starts here
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
// A far cold start the TMAGs disown (the consistency test) is a zero error
// at the MMC, not a probe: the next try waits this long, not MAGLOC_FAR_RETRY_MS.
#define MAGLOC_PHANTOM_RETRY_MS 2000
#define MAGLOC_SEEN_MT 0.04f
#define MAGLOC_FAINT_MT 0.015f
// A sensor is QUIET - nothing there for it, its zero free to follow its
// drift and its absorb clock stopped - under this fraction of its faint
// level: half, 0.0075 mT for a TMAG5273, above its smoothed noise on the
// bench (about 0.0046 mT in magnitude; a quarter, 0.00375, sat under it,
// so the TMAGs were never quiet there and their clocks ran on whatever
// kept presence on).
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

// A fit from a cold start (no previous fix to start from) tries seven starting
// points and measures about 50 ms on the CH32H417; a tracking fit is two or
// three iterations. So while there is no fix, a cold start is only attempted
// this often, and the loop stays free for everything else.
#define MAGLOC_COLD_START_PERIOD_MS 200
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
#define MAGLOC_STEADY_ITERATIONS 2 // the default of "fit iters" (~0.85 ms each on the CH32H417). tools/hostsim/pencil.cpp: 2 a frame tracks exactly as the natural caps do (error, lag and jitter to the last digit); at 1 it loses 0.1 % of frames. More only makes the pulse the supply shows bigger (2026-09-20).

// While tracking, this many frames in a row may fail to fit before the magnet
// is called lost and the (slow, paced) cold start takes over. Each failed frame
// has already cost a cold start inside the solver, so keep it small.
#define MAGLOC_MAX_MISSES 2

// Worst residual/signal ratio still called a fix (see magFitSolve). Far from
// the array a real magnet fits to a few percent. Close in it does not: the
// misfit is about 3 x (sensor position error / distance), so a magnet 10 mm
// from two sensors at once turns half a millimetre of table error into 15 %.
// A rough fix that says how rough it is (fix.misfit) beats no fix.
#define MAGLOC_MAX_MISFIT 0.40f

// A fix whose own error bar (all three axes, 1 sigma) is wider than this is not
// offered as a fix at all.
#define MAGLOC_MAX_ERROR_MM 15.0f

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
#define MAGLOC_MAGNET_STRENGTH 4232.0f // the probe's of 2026-09-18 (tools/magcal found 4213 +/- 4.5 % on the 2026-09-21 recording)
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
#define MAGLOC_AXIS_PRIOR_MT 0.02f
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
#define MAGLOC_LEARN_MIN_SENSORS 5
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
#define MAGLOC_TIP_OFFSET_MM 0.0f
#define MAGLOC_MAGNET_ANGLE_DEG 0.0f

// Where the probe POINTS: carry the point on down the shaft to the breadboard's
// surface, this high above the sensors, and no further (the tracker's cursor,
// MagTracker.h). With the point resting on the board that is the point itself;
// held above it, it is the hole the probe is aimed at rather than the one
// under it. 0 = not known: the sensor plane is used. The row calibration
// (`c`) sets it from the height the point rests at; `S` types it. On the
// bench the breadboard sits 17.5 mm over the sensors; on V6 the surface is
// 7.1 mm above the base PCB (plus wherever the sensors sit below that).
#define MAGLOC_BOARD_Z_MM 17.5f
// Where the point bottoms out is also WATCHED: each second the lowest point
// of the tracked fixes is noted, and when the last MAGLOC_FLOOR_WINDOWS of
// them agree within MAGLOC_FLOOR_AGREE_MM that is the floor (floorZ), which
// is reported against the surface as set. It is NOT taken as the surface
// any more: the point bottoms out in a hole, a lying magnet at its radius,
// and with the tip offset wrong the "point" is wherever the magnet is
// (2026-09-19 the rule brought the surface 6.7 mm down under a probe whose
// magnet was that far up the shaft; 2026-09-21 it put the surface 4 mm
// under a resting probe and the pointed cursor projected the difference).
// S and the menu set the surface; q/Q measure the tip offset.
#define MAGLOC_FLOOR_WINDOW_MS 1000
#define MAGLOC_FLOOR_WINDOWS 3
#define MAGLOC_FLOOR_AGREE_MM 0.7f
#define MAGLOC_FLOOR_STEP_MM 0.5f
#define MAGLOC_FLOOR_MISFIT 0.10f // a fix worse than this says nothing about the floor

// A fit is offered to the tracker as a ROUGH fix ("somewhere about here",
// the wide glow on the LEDs, no row counted) rather than a proper one when
// its error bar is wider than MAGLOC_ROUGH_ABOVE_MM - a probe far up or off
// the edge - and a fit that is not good enough to be a fix at all is still
// offered as rough if its misfit and bar are under the MAX limits, with the
// bar never claimed narrower than the lattice search resolves.
#define MAGLOC_ROUGH_ABOVE_MM 5.0f
#define MAGLOC_ROUGH_MAX_MISFIT 0.7f
#define MAGLOC_ROUGH_MAX_ERROR_MM 60.0f
#define MAGLOC_ROUGH_MIN_SIGMA_MM 3.0f
// The FAR probe - read plainly by the MMC56x3 alone, fewer than
// MAGLOC_MIN_SENSORS noticing it - is told from a zero error at the MMC by
// one thing only: how well a dipole of the held strength explains what is
// read. The probe 60 mm up fits at 4 % misfit with a 6 mm bar, 90 mm up
// with an 18 mm bar; a 0.09 mT zero error at the MMC (2026-09-21: its saved
// zero put back under a newly calibrated table row) "fitted" at 40-70 %
// with a 30-90 mm bar several times a second, and every such fit was a
// rough fix - the far glow at the board's centre - and reset the absorb
// clock, so the error was never absorbed and every near fix leaned on it.
// So a fit resting on fewer than MAGLOC_MIN_SENSORS is a fix, or a rough
// one, only under these (tighter than MAGLOC_MAX_MISFIT and
// MAGLOC_MAX_ERROR_MM, which are for a fit the TMAGs take part in); one
// that does not make them is nothing fitting, and the absorb rule takes
// what the MMC reads into its zero in MAGLOC_ROUGH_ONLY_ABSORB_MS.
#define MAGLOC_FAR_MISFIT 0.15f  // (kept for the report; the far acceptance is MAGLOC_FAR_MAX_CHI since 2026-09-22)
#define MAGLOC_FAR_ERROR_MM 30.0f // (25 until 2026-09-22: with the bench's zero errors the bar at 80 mm is 24, and half the frames fell out)
// The far acceptance, since 2026-09-22 morning: not a relative misfit but
// the residuals against what each sensor is expected to be off by
// (magSensorErrorMt: noise, zero uncertainty, the model's share) - the rms
// of residual over expected error, over the sensors that voted, under this.
// A TMAG reading its own zero error 80 mm from the probe is then a reading
// the model explains within its error, not a failure of the fit (the bench:
// far fixes rejected at a 13-15 % misfit, one a second, "slow and choppy").
#define MAGLOC_FAR_MAX_CHI 2.0f
// ...and CONFIRMED by the TMAGs when it can be: a probe within ~65 mm puts
// a definite pattern of 0.01-0.04 mT on them, a zero error at the MMC puts
// nothing. Over the TMAGs that voted, the gain of readings on predictions
// (sum read.pred / sum pred.pred) is near 1 for a probe and near 0 for a
// phantom; it is asked for when the predicted pattern's rms is above
// MAGLOC_CONFIRM_MIN_MT (a TMAG's noise), and must be over
// MAGLOC_CONFIRM_MIN_GAIN. Only a confirmed fix (or a near one) clears
// the absorb clocks: an unconfirmable far fix (beyond ~65 mm) is real or
// not, and only the clocks can say - a still one is absorbed in time, a
// moving probe is not (MAGLOC_STEADY_FRACTION). (2026-09-22: judged by
// residuals alone, the post-boot 0.12 mT zero error at the MMC fitted as a
// probe 43 mm up, its bar cleared the clocks, and it was never absorbed:
// 542 cold starts in three minutes with nothing there.)
#define MAGLOC_CONFIRM_MIN_MT 0.012f
#define MAGLOC_CONFIRM_MIN_GAIN 0.5f
// The fast absorb runs only on a sensor whose reading has been steady -
// the reading AS MEASURED, before the baseline, smoothed, within this
// fraction (of the baseline-removed reading's size) of a slow average of
// itself: a zero error is static, a probe in a hand moves. (Judged on the
// baseline-removed reading it ran in bursts, the absorb itself unsteadying
// the sensor: a probe lying beside the board flapped the far track once a
// second for twelve minutes, 2026-09-22 evening.)
#define MAGLOC_STEADY_FRACTION 0.10f
#define MAGLOC_STEADY_ALPHA 0.02f
// ...and a far track rides through this many frames that do not make it
// (the near track's MAGLOC_MAX_MISSES is 2; far, at the edge of reach, a
// frame in five fails on noise), starting each from the last good place.
#define MAGLOC_FAR_MAX_MISSES 100 // a second: the rejections come in runs, the smoothed fields change slowly
// A track lost moments ago is looked for first where it last was - one warm
// fit from the last accepted fix - before any lattice search: the lattice
// has no memory, and for a far probe it lands as readily in the mirror
// basin (the same three numbers at the MMC from a magnet 54 mm off the
// side of the board, on the sensor plane) as in the true one; the bench
// hopped between the two 112 times in a minute (2026-09-22 morning). Every
// frame for this long, for a far track (a near one is found again by its
// own misses and a lattice that is not ambiguous).
#define MAGLOC_REACQUIRE_MS 3000
// ...and a cold start that ended in such a rejection is not tried again for
// this long: a zero error at the MMC is present until the absorb rule has
// taken it (minutes), and a lattice search every 200 ms meanwhile (3.7 a
// second on the bench, 2026-09-21) is the V5F's heaviest work for nothing.
// A probe that then arrives is found at the next try, within a second.
#define MAGLOC_FAR_RETRY_MS 400 // (1000 until 2026-09-22: a second between tries was the "slow" of a far fix at the edge)

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

// While nothing at all is read (the strongest smoothed reading under
// MAGLOC_FAINT_MT), the baseline follows the readings with this fraction per
// frame (0.0005 = a 20 s time constant at 100 Hz), so the slow drift of the
// ambient field and the sensors' offsets (0.1 mT in ten minutes was seen on
// the bench, 0.0002 mT/s, which a 20 s constant follows with 0.003 mT of
// lag) does not become a phantom magnet - and not at all within
// MAGLOC_DRIFT_HOLDOFF_MS of a magnet having been present: a probe at the
// edge of reach dips under the faint level as it wavers, and a 5 s constant
// was eating its field before it came back (2026-09-18: "fades out at the
// outer range"). A magnet that reads more than the faint level is never
// absorbed by this rule.
#define MAGLOC_BASELINE_DRIFT 0.0005f
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
#define MAGLOC_AUDIT_MIN_SEEN 6
#define MAGLOC_AUDIT_MAX_ERROR_MM 3.0f
#define MAGLOC_AUDIT_MIN_SAMPLES 40
#define MAGLOC_AUDIT_MAX_SAMPLES 400 // a window that never crosses the threshold starts again after this many (an hour's drift is not a week's)
#define MAGLOC_AUDIT_MIN_SPREAD 0.3f
#define MAGLOC_AUDIT_APPLY_MT 0.004f         // ...or 0.6 of the sensor's own noise, whichever is more (a TMAG: 0.008)
#define MAGLOC_AUDIT_APPLY_NOISE 0.6f
#define MAGLOC_AUDIT_MAX_GAIN_ERROR 0.15f    // a sensor whose gain solves further from 1 than this...
#define MAGLOC_AUDIT_MAX_UNEXPLAINED 0.15f   // ...or whose residual after the solve is more than this of its field, is left out of the audit's fits
#define MAGLOC_AUDIT_MAX_MT 0.3f
#define MAGLOC_AUDIT_ITERATIONS 6
#define MAGLOC_ZERO_SETTLE_MT 0.01f
// ...and not more often than this: a settle is a settings write (a page
// erase, the LED chain blinked off for it), and a zero that only matters at
// the next boot can wait. The first after a boot does not wait.
#define MAGLOC_ZERO_SETTLE_MIN_MS 600000
#define MAGLOC_DRIFT_HOLDOFF_MS 10000
// ...and also while something has been "present" for this long without ever
// fitting as a magnet (rough fixes only): that is drift or a stuck reading,
// not a probe - a probe fits. (Overnight on the bench, drift crossed the
// 0.04 mT presence line now and then and made 2,293 rough fixes in six
// hours with no magnet in the room.) Two minutes: long enough to hover at
// the edge, short enough that a phantom is gone before anyone looks.
#define MAGLOC_ROUGH_ONLY_ABSORB_MS 120000
// ...but a sensor that reads the magnet PLAINLY (above MAGLOC_SEEN_MT in its
// own terms) with nothing fitting for this long is a zero error, not a
// probe: a probe that gives the MMC56x3 0.04 mT is 60 mm off and the TMAGs
// see it too, and a fit of it works (the sim: 60 mm, misfit 4 %). Twenty
// seconds, so a boot with a stale record (2026-09-21: the MMC's zero read
// 0.12 mT with nothing there, and the probe put on the board in that state
// got no fix at all) is right in under a minute instead of three.
#define MAGLOC_SEEN_ABSORB_MS 20000
#define MAGLOC_SEEN_ABSORB_LEVEL ( 2.0f * MAGLOC_SEEN_MT ) // "plainly": twice the seen level (the MMC: 0.016 mT, a probe within 80 mm, which fits with a 10 mm bar)
// Once a sensor's clock has run out it is ABSORBING: its zero follows its
// reading at this fraction a frame (2 s, against 20 s for the slow drift)
// until it reads quiet, and a fix that only it makes does not stop that -
// the tail of a zero error at the MMC, 0.01 mT, IS a probe 90 mm up to
// three numbers with the strength held, and its fixes cleared the clocks
// and kept the tail alive. A fix the TMAGs take part in does stop it.
#define MAGLOC_ABSORB_FAST 0.005f

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
    Vec3 sigma;      // 1-sigma error bar on the magnet position, mm, per axis (from the fit itself)
    float errorXyMm; // ...across the board: sqrt(sx^2 + sy^2)
    float errorMm;   // ...and in all three axes together
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
    uint32_t coldWhyPhantom = 0; // far cold starts the TMAGs disowned
    uint32_t coldWhyAbsent = 0, coldWhyFew = 0, coldWhyMisses = 0, coldWhyRejected = 0; // what ended the track before each cold start: nothing present, too few noticing, the misses run out, a far cold start rejected
    uint32_t staleFrames[ MAGFIT_MAX_SENSORS ] = { 0 }; // frames in which a sensor was not read (the MMC56x3's 150 Hz against 100 Hz frames)
    uint32_t reacquired = 0, reacquireTries = 0; // times a lost track was found again from where it last was (MAGLOC_REACQUIRE_MS), and the frames that tried
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
    void forgetStrength( );        // K: never hold it (fit it freely)
    void restoreStrength( float strength ); // the settings' record at boot

    // How far the magnet's centre is up the shaft from the probe's point, and
    // the magnet's angle to the shaft.
    float tipOffsetMm = MAGLOC_TIP_OFFSET_MM;
    float magnetAngleDeg = MAGLOC_MAGNET_ANGLE_DEG;
    float boardZ = MAGLOC_BOARD_Z_MM; // the breadboard's surface, mm above the sensors (the menu's "surface", saved; learned too, see MAGLOC_FLOOR_*)
    float floorZ = 0.0f;              // where the point bottoms out, as learned (0 = not yet); the surface only ever comes UP to it
    float floorWindowMinZ = 1e9f;     // the lowest point this second
    uint32_t floorWindowStartMs = 0;
    float floorMins[ MAGLOC_FLOOR_WINDOWS ]; // the last seconds' lowest points
    int floorMinCount = 0;
    uint32_t surfaceLearned = 0; // times the surface came down to the floor
    void learnFloor( uint32_t nowMs );
    float presentMt = MAGLOC_PRESENT_MT; // the strongest smoothed reading that counts as a magnet (menu: presence)
    // The last good fix (a sharp one, misfit under MAGLOC_LEARN_MAX_MISFIT):
    // kept in the settings, so that after a reboot with the probe lying where
    // it was, `Y` can take that magnet back out of the baseline the boot
    // zeroed it into (unpolluteBaseline). Starts as the fix measured on
    // 2026-09-18 with the probe resting in row 35, hole 3, at 41 degrees.
    Vec3 lastGoodPosition = { MAGLOC_REFERENCE_X, MAGLOC_REFERENCE_Y, MAGLOC_REFERENCE_Z };
    Vec3 lastGoodAxis = { MAGLOC_REFERENCE_AX, MAGLOC_REFERENCE_AY, MAGLOC_REFERENCE_AZ };
    float lastGoodStrength = MAGLOC_REFERENCE_STRENGTH;
    bool haveLastGood = true;
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
    Vec3 recent[ MAGFIT_MAX_SENSORS ][ 2 ] = { }; // each sensor's last two frames, for the glitch filter
    int missedFrames[ MAGFIT_MAX_SENSORS ] = { }; // frames in a row each sensor was not read
    uint32_t lastFrame = 0;
    bool haveSmoothed = false;
    uint32_t nextColdStartMs = 0;
    uint32_t farRetryUntilMs = 0; // a far retry's wait, honoured in the steady load too
    int misses = 0; // frames in a row that would not fit, while tracking
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
    uint32_t roughOnlySinceMs = 0;         // when "present without a fit" began (0 = not now)
    uint32_t presentSinceMs[ MAGFIT_MAX_SENSORS ] = { 0 }; // since when each sensor has itself read something (above its quiet level) that no fix explains (0 = it has not); a sensor's own clock, which its dropping off the bus does not reset
    uint32_t lastPresentMs = 0;            // when a magnet was last present (the drift hold-off)
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
        bool quiet[ MAGFIT_MAX_SENSORS ];    // reading well under its own faint level: nothing there for it
        bool plainHere[ MAGFIT_MAX_SENSORS ]; // reading plainly: above MAGLOC_SEEN_ABSORB_LEVEL in its own terms
        uint32_t nowMs = 0;
        float weights[ MAGFIT_MAX_SENSORS ]; // each sensor's weight in the fit this frame
        float held = 0.0f;                   // the strength held (0 = free)
        int plain = 0;                       // sensors reading plainly (plainHere)
        bool hintFree = false;               // the free fit ran with no strength hint: the readings pinned it (a measurement)
        bool enough = false;                 // enough sensors notice it to try a fit
        bool farOnly = false;                // the fit rests on fewer than two sensors seeing it plainly
        bool good = false;                   // the fit's verdict
        bool confirmed = false;              // a near fix, or a far one the TMAGs confirm: may clear the absorb clocks
        bool steady[ MAGFIT_MAX_SENSORS ];   // the sensor's reading steady (MAGLOC_STEADY_FRACTION): the fast absorb may run on it
    };
    bool filterFrame( FrameScratch& f );
    void assessFrame( FrameScratch& f );
    void keepZeros( const FrameScratch& f );
    ServiceStatus tryCoarse( MagTrackInput* in, const FrameScratch& f );
    ServiceStatus runFit( MagTrackInput* in, FrameScratch& f );
    ServiceStatus publishFix( MagTrackInput* in, const FrameScratch& f );
    ServiceStatus fitFrame( MagTrackInput* in );
    ServiceStatus simFrame( MagTrackInput* in );
    bool offerRough( MagTrackInput* in, const MagFitResult* r ) const;
    void clearPresentClocks( );
    uint32_t lastZeroSettleMs = 0; // when the saved zeros last followed the drifted ones (MAGLOC_ZERO_SETTLE_MIN_MS) - unused since the audit took over the writing
    uint32_t lastAuditMs = 0;      // the zero audit's last sample
    Vec3 lastFixPosition = { 0, 0, 0 }, lastFixMoment = { 0, 0, 0 }; // the last accepted fix (rough included), for MAGLOC_REACQUIRE_MS
    uint32_t lastFixMs = 0;
    bool lastFixFar = false;       // ...and whether it was far-sized (one plain sensor, or a bar the near rule would not take): only a far track is re-acquired this way
    int auditNext = 0;             // the sensor it looks at next (round robin)
    void auditZeros( const FrameScratch& f );
    float chiOf( const MagFitResult& r, const bool* use ) const; // the rms of residual over expected error over the sensors that voted
    void tmagConsistency( const MagFitResult& r, const bool* use, float* gain, float* predRms ) const; // the TMAGs' readings on the fit's predictions
    Vec3 rawSmooth[ MAGFIT_MAX_SENSORS ] = { }; // each reading as measured (before the baseline), smoothed like `smooth`
    Vec3 slowRef[ MAGFIT_MAX_SENSORS ] = { };   // a slow average of rawSmooth, for MAGLOC_STEADY_FRACTION
    bool absorbing[ MAGFIT_MAX_SENSORS ] = { false }; // this sensor's zero is following its reading fast (MAGLOC_ABSORB_FAST) until it reads quiet // true if it was offered

    Vec3 pointerOf( Vec3 tip, Vec3 shaft ) const;
    void printFixCsv( Stream* out ) const;
};

extern MagLocator& magLocator;

#endif // MAGLOCATOR_H
