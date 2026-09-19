// Soak: 10 minutes of a random hand over the array through the real locator + tracker + row counter, looking for NaNs, stuck tracks and mis-states.
#define private public
#include "MagArray.h"
#include "MagLocator.h"
#include "RowCounter.h"
#include "ProbeLedService.h"
#undef private
#include "Console.h"


static float gauss(float s){ float u1=(rand()+1.0f)/((float)RAND_MAX+2.0f), u2=(rand()+1.0f)/((float)RAND_MAX+2.0f); return s*sqrtf(-2*logf(u1))*cosf(2*M_PI*u2);}
static float frand(float a, float b){ return a + (b-a)*rand()/(float)RAND_MAX; }
static bool bad(float v){ return v != v || v > 1e6f || v < -1e6f; }
int main(){ console.begin(&Serial); magArray.begin(); magArray.useSimulatedFrames();
  magLocator.begin(); rowCounter.begin(); probeLeds.begin(); srand(11);
  Vec3 p={27,22,25}, v={0,0,0}, goal=p; Vec3 shaft={0,0,1}; float lean=0.2f, az=0;
  long frames=0, nanCount=0, lostInRange=0, roughInRange=0, tracking=0, coasting=0, rough=0, none=0, dropped=0, reinits=0; double err2=0; long errN=0; float worst=0;
  for (int f=0; f<60000; f++) { // 10 min
    // hand: pick goals, move with bounded accel, occasional dwell
    if (f % 300 == 0) { goal = { frand(-15, 70), frand(-15, 60), frand(17.5f, 45) }; if (rand()%4==0) goal = p; lean = frand(0, 0.5f); az = frand(0, 6.28f); }
    Vec3 d={goal.x-p.x, goal.y-p.y, goal.z-p.z}; float dist=sqrtf(d.x*d.x+d.y*d.y+d.z*d.z);
    float acc = 3000.0f; // mm/s^2
    for (int a=0;a<3;a++){ float* pv=a==0?&v.x:(a==1?&v.y:&v.z); float dd=a==0?d.x:(a==1?d.y:d.z); float want = dist>0? dd/dist*fminf(400.0f, dist*4.0f):0; *pv += fmaxf(-acc*0.01f, fminf(acc*0.01f, want-*pv)); }
    p={p.x+v.x*0.01f, p.y+v.y*0.01f, p.z+v.z*0.01f}; if (p.z<17.5f) p.z=17.5f;
    shaft={sinf(lean)*cosf(az), sinf(lean)*sinf(az), cosf(lean)};
    bool dropout = (f % 1000) > 985; // 140 ms every 10 s
    simMicros += 10000; Vec3 m = { 4200*shaft.x, 4200*shaft.y, 4200*shaft.z };
    for (int i=0;i<MAG_SENSOR_COUNT;i++){ magArray.field[i]=magFitDipoleField(magArray.position[i],p,m); magArray.field[i].x+=gauss(0.005f)+ (rand()%40==0? gauss(0.3f):0); magArray.field[i].y+=gauss(0.005f); magArray.field[i].z+=gauss(0.005f); magArray.fresh[i]=!dropout; }
    magArray.frameCount++; magLocator.service(); rowCounter.service(); probeLeds.lastUs=0; probeLeds.service();
    const MagTrack& t = magLocator.track; frames++;
    if (bad(t.position.x)||bad(t.cursor.x)||bad(t.sigma.x)||bad(t.shaft.x)||bad(magLocator.fix.magnet.x)) nanCount++;
    bool inRange = p.x>-5 && p.x<60 && p.y>-5 && p.y<50 && p.z<32;
    switch(t.state){ case MAGTRACK_TRACKING: tracking++; break; case MAGTRACK_COASTING: coasting++; break; case MAGTRACK_ROUGH: rough++; if(inRange) roughInRange++; break; default: none++; if(inRange && !dropout) lostInRange++; }
    if (t.state==MAGTRACK_TRACKING && inRange) { float e=sqrtf(powf(t.position.x-p.x,2)+powf(t.position.y-p.y,2)+powf(t.position.z-p.z,2)); err2+=e*e; errN++; if(e>worst) worst=e;
      static int shown=0; if (e>8 && shown<12) { shown++; const MagProbeFix& fx=magLocator.fix; float fe=sqrtf(powf(fx.rawMagnet.x-p.x,2)+powf(fx.rawMagnet.y-p.y,2)+powf(fx.rawMagnet.z-p.z,2));
        printf("  f%d truth %.1f %.1f %.1f speed %.0f | fix %.1f %.1f %.1f err %.1f valid %d misfit %.2f bar %.1f | track err %.1f sigma %.1f gate %.1f dropped %d\n", f, p.x,p.y,p.z, sqrtf(v.x*v.x+v.y*v.y+v.z*v.z), fx.rawMagnet.x,fx.rawMagnet.y,fx.rawMagnet.z, fe, fx.valid, fx.misfit, fx.errorMm, e, t.sigma.x, t.lastGate, t.lastDropped); } }
    for (int i=0;i<probeLeds.frame.count;i++) if (bad(probeLeds.frame.level[i])) nanCount++;
  }
  const MagTrack& t = magLocator.track;
  printf("soak: %ld frames, NaN/huge %ld, states tracking %ld coasting %ld rough %ld none %ld; none while in range %ld, rough while in range %ld; track error in range %.2f mm rms worst %.1f (%ld); dropped %lu reinits %lu\n",
    frames, nanCount, tracking, coasting, rough, none, lostInRange, roughInRange, sqrt(err2/(errN?errN:1)), worst, errN, (unsigned long)t.dropped, (unsigned long)t.reinits);
  return 0; }
