// SPDX-License-Identifier: MIT
// Host scan of the gradient-tensor estimate (MagFar) against distance and noise; the numbers in attic/README.md.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "MagFar.h"
#include "MagFit.h"
static const Vec3 sensors[8] = {
    { 0.10f, 44.29f, 0 }, { 16.75f, 44.04f, 0 }, { 38.78f, 43.95f, 0 }, { 54.18f, 44.72f, 0 },
    { 0.00f, 0.00f, 0 },  { 15.60f, 0.19f, 0 },  { 37.37f, 0.42f, 0 },  { 53.40f, 0.00f, 0 } };
static float gaussian(float s){ float u1=(rand()+1.0f)/((float)RAND_MAX+2.0f), u2=(rand()+1.0f)/((float)RAND_MAX+2.0f); return s*sqrtf(-2*logf(u1))*cosf(2*M_PI*u2);}
int main(){
  srand(3);
  // error as a fraction of distance, binned by distance from the array centre, for a 4200 magnet at random tilts
  const float noises[3] = {0.0f, 0.005f, 0.02f};
  for (int ni = 0; ni < 3; ni++) { float noise = noises[ni];
    printf("noise %.3f mT\n", noise);
    for (float d0 = 20; d0 <= 120; d0 += 10) {
      int n=0, valid=0; double sum=0, sumsq=0, worst=0;
      for (int trial=0; trial<400; trial++) {
        // random point at distance in [d0, d0+10) from centre (27,22,0), above the board
        float d = d0 + 10.0f*rand()/(float)RAND_MAX;
        float az = 2*M_PI*rand()/(float)RAND_MAX, el = acosf(1.0f - 1.0f*rand()/(float)RAND_MAX); // hemisphere
        Vec3 magnet = { 27.0f + d*sinf(el)*cosf(az), 22.0f + d*sinf(el)*sinf(az), d*cosf(el) };
        if (magnet.z < 0.7f*d) continue;
        float t = 60.0f*M_PI/180*rand()/(float)RAND_MAX, h = 2*M_PI*rand()/(float)RAND_MAX;
        Vec3 moment = { 4200*sinf(t)*cosf(h), 4200*sinf(t)*sinf(h), -4200*cosf(t) };
        Vec3 fields[8];
        for (int i=0;i<8;i++){ fields[i]=magFitDipoleField(sensors[i],magnet,moment); fields[i].x+=gaussian(noise); fields[i].y+=gaussian(noise); fields[i].z+=gaussian(noise);}
        MagFarEstimate e; magFarEstimate(sensors, fields, nullptr, 8, 4200, &e);
        n++;
        if (!e.valid) continue;
        valid++;
        float miss = sqrtf(powf(e.position.x-magnet.x,2)+powf(e.position.y-magnet.y,2)+powf(e.position.z-magnet.z,2));
        float frac = miss / d; sum += frac; sumsq += frac*frac; if (frac>worst) worst=frac;
      }
      if (valid) printf("  d %3.0f-%3.0f: valid %3d/%3d  miss/d mean %.3f rms %.3f worst %.3f\n", d0, d0+10, valid, n, sum/valid, sqrt(sumsq/valid), worst);
    }
  }
  // strongest field at the centre vs distance for 4200
  return 0;
}
