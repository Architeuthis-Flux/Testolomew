// SPDX-License-Identifier: MIT
#ifndef ZERO_AUDIT_H
#define ZERO_AUDIT_H
// ---------------------------------------------------------------------------
// The zero audit: each sensor's zero (and gain) checked against the fit of
// the OTHERS while the probe is near.
//
// One sensor cannot tell its own zero error from a far probe - to three
// numbers with the strength held, 0.01 mT at the MMC56x3 IS the probe 90 mm
// up - and every rule that tried (absorb after so long, settle what drifted)
// ate a real hover once and made it permanent (2026-09-21: the MMC's saved
// zero moved 0.1 mT while the probe was waved about). What CAN tell is the
// rest of the array when the probe is near: eight TMAG5273s fit the magnet
// to a fraction of a millimetre, and that fit predicts what the ninth sensor
// should read. Reading minus prediction, over many probe positions, is that
// sensor's own error: a constant part (its zero) and a part that scales with
// the field (its gain). Both are solved together, because a 3 % gain error
// looks exactly like a zero error at one position and an offset-only audit
// would chase it from position to position.
//
// The locator feeds it: while the fit is good and the TMAGs see the magnet
// plainly, once a second it refits without one sensor (round robin) and hands
// over that sensor's reading and the prediction. When a sensor has enough
// samples over enough spread, the solve gives its zero z and gain g; a zero
// past the apply threshold is taken out of the baseline (MagArray::
// shiftBaseline, which the settings then keep) and logged. The gain is
// reported, not applied: a gain that is not 1 is a table row to look at
// (magcal), or a sensor reading something the dipole is not.
//
// The model, per sensor, per sample k, per axis a:
//   reading_ka = z_a + g * pred_ka
// Least squares over all k and a: with N samples, R_a = sum reading, P_a =
// sum pred, PP_a = sum pred^2, PR_a = sum pred*reading,
//   g = sum_a (PR_a - R_a P_a / N) / sum_a (PP_a - P_a^2 / N)
//   z_a = (R_a - g P_a) / N
// The denominator is N times the spread of the prediction over the samples:
// with the probe held still it is zero and z and g cannot be told apart, so
// the solve waits for spread (minSpread: the rms spread of the prediction
// over its mean).
//
// `:audit` prints the table: samples, spread, z, g, the fit's rms before and
// after, and how many times each sensor's zero was corrected.
// ---------------------------------------------------------------------------
#include <stdint.h>

#include "Vec3.h"

#define ZERO_AUDIT_MAX 16

struct ZeroAuditSensor {
    int n = 0;
    double r[ 3 ] = { 0, 0, 0 };  // sum of the readings, per axis
    double p[ 3 ] = { 0, 0, 0 };  // sum of the predictions
    double pp[ 3 ] = { 0, 0, 0 }; // sum of prediction^2
    double pr[ 3 ] = { 0, 0, 0 }; // sum of prediction * reading
    double rr[ 3 ] = { 0, 0, 0 }; // sum of reading^2 (for the rms before)
    uint32_t firstMs = 0;
    // The last solve.
    bool solved = false;
    Vec3 zero = { 0, 0, 0 };
    float gain = 1.0f;
    float spread = 0.0f;   // rms spread of |pred| over its mean
    float rmsBefore = 0.0f, rmsAfter = 0.0f; // of the residual, mT
    int applied = 0;       // times this sensor's zero was corrected
    bool excluded = false; // the model does not describe this sensor (a gain far from 1, or a residual it cannot explain): left out of the audit's fits of the others
    Vec3 lastApplied = { 0, 0, 0 };
};

class ZeroAudit {
  public:
    void reset( int i );
    void resetAll( );
    void addSample( int i, Vec3 prediction, Vec3 reading, uint32_t nowMs );
    // Solve sensor i's zero and gain from its samples; false when there are
    // too few samples or too little spread to tell the two apart. The solve
    // is left in the sensor's record either way (for the report).
    bool solve( int i, int minSamples, float minSpread );
    const ZeroAuditSensor& sensor( int i ) const { return s[ i ]; }
    ZeroAuditSensor& sensor( int i ) { return s[ i ]; }

  private:
    ZeroAuditSensor s[ ZERO_AUDIT_MAX ];
};

#endif // ZERO_AUDIT_H
