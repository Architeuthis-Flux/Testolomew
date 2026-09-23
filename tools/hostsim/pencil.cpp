// Pencil: how the track, the scene's magnet and the cursor follow a hand that
// writes - strokes of 50-250 mm/s with pauses, the pencil turning as it
// goes - through the real locator and tracker. Reports the error and the
// lag (the delay of the truth that best matches each output) of each
// output, the shaft's angle error at rest and while turning, and the
// jitter at rest. The numbers to tune the smoothing levers by.
#include "MagArray.h"
#include "MagLocator.h"
#include "MagTracker.h"
#include "Console.h"
#include <vector>
#include <string.h>
#include <stdlib.h>


static float gauss(float s){ float u1=(rand()+1.0f)/((float)RAND_MAX+2.0f), u2=(rand()+1.0f)/((float)RAND_MAX+2.0f); return s*sqrtf(-2*logf(u1))*cosf(2*M_PI*u2);}
static float frand(float a, float b){ return a + (b-a)*rand()/(float)RAND_MAX; }
struct Sample { Vec3 truth, shaftTruth, track, view, cursorTruth, cursor; Vec3 shaft; bool tracking, moving, turning; };
static float dist(Vec3 a, Vec3 b){ return sqrtf((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)); }
static float deg(Vec3 a, Vec3 b){ float d=a.x*b.x+a.y*b.y+a.z*b.z; if(d>1)d=1; if(d<-1)d=-1; return acosf(d)*180/M_PI; }
// best lag (frames) of `out` behind `truth`, over moving samples
static void lagOf(const std::vector<Sample>& s, Vec3 Sample::*truth, Vec3 Sample::*out, const char* name){
  float bestErr=1e9; int bestLag=0;
  for (int lag=0; lag<=30; lag++){ double e2=0; long n=0; for (size_t i=lag; i<s.size(); i++){ if(!s[i].tracking||!s[i-lag].moving) continue; float e=dist(s[i].*out, s[i-lag].*truth); e2+=e*e; n++; } float e=n?sqrtf(e2/n):1e9; if(e<bestErr){bestErr=e; bestLag=lag;} }
  double e0=0; long n0=0; for (size_t i=0;i<s.size();i++){ if(!s[i].tracking||!s[i].moving) continue; float e=dist(s[i].*out, s[i].*truth); e0+=e*e; n0++; }
  printf("  %-8s moving: error %.2f mm rms against the truth now, %.2f against the truth %d frames (%d ms) ago (the lag)\n", name, n0?sqrt(e0/n0):0, bestErr, bestLag, bestLag*10);
}
int main(int argc, char** argv){ console.begin(&Serial); magArray.begin(); magArray.useSimulatedFrames();
  magLocator.begin(); srand(5);
  MagTrack& tr = magLocator.track;
  // levers from the command line: viewHz viewBeta cursorHz cursorBeta shaftHz shaftBeta [accel]
  if (argc>=7){ tr.viewMinCutoff=atof(argv[1]); tr.viewBeta=atof(argv[2]); tr.oneEuroMinCutoff=atof(argv[3]); tr.oneEuroBeta=atof(argv[4]); tr.shaftMinCutoff=atof(argv[5]); tr.shaftBeta=atof(argv[6]); }
  if (argc>=8) tr.accelSigma=atof(argv[7]);
  if (argc>=9) magLocator.speedJitterK=atof(argv[8]);
  // "steady N" as the last two arguments: the fit on a steady budget of N iterations a frame (MagLocator's fit load), to see what it costs in error and lag.
  if (argc>=3 && strcmp(argv[argc-2], "steady")==0) { magLocator.steadyFit = true; magLocator.steadyIterations = atof(argv[argc-1]); argc -= 2; printf("fit load: steady, %.0f iterations a frame\n", magLocator.steadyIterations); }
  bool far = argc>=2 && strcmp(argv[argc-1], "far")==0; // hovering 12-22 mm above the surface instead of writing on it: the weak-field smoothing is on
  float zLow = far ? 30.0f : 17.5f, zHigh = far ? 40.0f : 24.0f;
  if (far) printf("far: hovering %.0f-%.0f mm up\n", zLow, zHigh);
  printf("levers: view %.2f Hz / beta %.3f, cursor %.2f / %.3f, shaft %.2f / %.2f, accel %.0f, speed jitter K %.2f\n", tr.viewMinCutoff, tr.viewBeta, tr.oneEuroMinCutoff, tr.oneEuroBeta, tr.shaftMinCutoff, tr.shaftBeta, tr.accelSigma, magLocator.speedJitterK);
  magLocator.tipOffsetMm = 0; tr.tipOffsetMm = 0; tr.cursorMode = MAGCURSOR_POINTED;
  // MAG_BIAS=i,x,y,z in the environment: a zero error at sensor i (mT, board frame) on every frame
  if (getenv("MAG_WCAP")) { magArray.setWeightCap(atof(getenv("MAG_WCAP"))); printf("weight cap %s\n", getenv("MAG_WCAP")); }
  if (getenv("MAG_FREE")) { magLocator.forgetStrength(); printf("strength free (not held)\n"); }
  // MAG_STRENGTH=<mT*mm^3>, MAG_NOISE=x,y,z (a TMAG's per-axis noise a frame), MAG_MMC=0 (the MMC56x3 out): the bench as it is (1839, out) against the defaults the levers were set by (4200, in)
  float strength = getenv("MAG_STRENGTH") ? atof(getenv("MAG_STRENGTH")) : 4200.0f;
  Vec3 noise = {0.012f, 0.012f, 0.006f}; if (getenv("MAG_NOISE")) { if (sscanf(getenv("MAG_NOISE"), "%f,%f,%f", &noise.x, &noise.y, &noise.z)!=3) noise = {0.012f, 0.012f, 0.006f}; } // the bench's anisotropy (isotropic 0.010 until 2026-09-23)
  if (getenv("MAG_MMC") && atoi(getenv("MAG_MMC"))==0) magArray.useMmc = false;
  printf("world: magnet %.0f, noise %.3f %.3f %.3f mT a frame, the MMC %s\n", strength, noise.x, noise.y, noise.z, magArray.useMmc ? "in" : "out");
  bool biasOn=false; int biasI=0; Vec3 bias={0,0,0}; if (getenv("MAG_BIAS")) { biasOn = sscanf(getenv("MAG_BIAS"), "%d,%f,%f,%f", &biasI, &bias.x, &bias.y, &bias.z)==4; if (biasOn) printf("bias: sensor %d %.4f %.4f %.4f mT\n", biasI, bias.x, bias.y, bias.z); }
  Vec3 p={27,22,zLow+2.5f}, v={0,0,0}, goal=p; float lean=0.3f, az=1.0f, leanGoal=0.3f, azGoal=1.0f; int dwell=0;
  std::vector<Sample> samples; double restSpeed=0, restAlpha=0, moveSpeed=0; long restN=0, moveN=0;
  for (int f=0; f<30000; f++) { // 5 min
    if (f % 150 == 0) { // a new stroke every 1.5 s, a pause one time in three
      if (rand()%3==0) { goal=p; dwell=150; } else { goal = { frand(5, 50), frand(5, 40), frand(zLow, zHigh) }; dwell=0; leanGoal = frand(0.1f, 0.7f); azGoal = frand(0, 6.28f); }
    }
    Vec3 d={goal.x-p.x, goal.y-p.y, goal.z-p.z}; float dd=sqrtf(d.x*d.x+d.y*d.y+d.z*d.z);
    for (int a=0;a<3;a++){ float* pv=a==0?&v.x:(a==1?&v.y:&v.z); float da=a==0?d.x:(a==1?d.y:d.z); float want = dd>0? da/dd*fminf(250.0f, dd*5.0f):0; *pv += fmaxf(-40.0f, fminf(40.0f, want-*pv)); }
    p={p.x+v.x*0.01f, p.y+v.y*0.01f, p.z+v.z*0.01f}; if (p.z<zLow) p.z=zLow;
    float turn = 1.0f*0.01f; // 1 rad/s at most
    lean += fmaxf(-turn, fminf(turn, leanGoal-lean)); float daz=azGoal-az; while(daz>M_PI)daz-=2*M_PI; while(daz<-M_PI)daz+=2*M_PI; az += fmaxf(-turn, fminf(turn, daz));
    Vec3 shaft={sinf(lean)*cosf(az), sinf(lean)*sinf(az), cosf(lean)};
    bool moving = sqrtf(v.x*v.x+v.y*v.y+v.z*v.z) > 5.0f; bool turning = fabsf(leanGoal-lean)>0.02f || fabsf(daz)>0.02f;
    simMicros += 10000; Vec3 m = { strength*shaft.x, strength*shaft.y, strength*shaft.z };
    for (int i=0;i<MAG_SENSOR_COUNT;i++){ float k = magArray.noiseMt[i]/MAG_WEIGHT_REFERENCE_MT; magArray.field[i]=magFitDipoleField(magArray.position[i],p,m); magArray.field[i].x+=gauss(noise.x*k); magArray.field[i].y+=gauss(noise.y*k); magArray.field[i].z+=gauss(noise.z*k); magArray.fresh[i]=true; }
    if (biasOn) { magArray.field[biasI].x+=bias.x; magArray.field[biasI].y+=bias.y; magArray.field[biasI].z+=bias.z; }
    magArray.frameCount++; magLocator.service();
    // the cursor's truth: down the shaft to the surface plane (17.5), as the tracker defines it
    Vec3 ct = p; { float reachT = 0; ct = magTrackPointer(magLocator.boardZ, MAGTRACK_MAX_REACH_MM, p, shaft, &reachT); } // the cursor's truth: the same geometry the tracker uses (touching = the point, else the pointer)
    Sample s = { p, shaft, tr.position, tr.viewPosition, ct, tr.cursor, tr.shaft, tr.state==MAGTRACK_TRACKING, moving, turning };
    samples.push_back(s);
    if (!moving && !turning && dwell>0) { restSpeed += magLocator.smoothSpeed; restAlpha += magLocator.smoothAlpha; restN++; } else if (moving) { moveSpeed += magLocator.smoothSpeed; moveN++; }
  }
  long tracking=0; for (auto& s: samples) tracking += s.tracking;
  printf("pencil: %zu frames, tracking %.1f %%\n", samples.size(), 100.0*tracking/samples.size());
  lagOf(samples, &Sample::truth, &Sample::track, "track");
  lagOf(samples, &Sample::truth, &Sample::view, "view");
  lagOf(samples, &Sample::cursorTruth, &Sample::cursor, "cursor");
  // rest: jitter (sd around the mean over each dwell) and error
  double jt=0, jv=0, jc=0; long jn=0; double angRest=0, angTurn=0; long nr=0, nt=0; double et=0, ev=0; long ne=0;
  for (size_t i=20;i<samples.size();i++){ const Sample& s=samples[i]; if(!s.tracking) continue;
    float a=deg(s.shaft, s.shaftTruth); if (s.turning) { angTurn+=a*a; nt++; } else if (!s.moving) { angRest+=a*a; nr++; }
    if (!s.moving && !s.turning) { et+=powf(dist(s.track,s.truth),2); ev+=powf(dist(s.view,s.truth),2); ne++;
      if (i>0 && !samples[i-1].moving) { jt+=powf(dist(s.track,samples[i-1].track),2); jv+=powf(dist(s.view,samples[i-1].view),2); jc+=powf(dist(s.cursor,samples[i-1].cursor),2); jn++; } } }
  printf("  at rest: track error %.2f mm rms, view %.2f; frame-to-frame jitter track %.3f mm, view %.3f, cursor %.3f\n", ne?sqrt(et/ne):0, ne?sqrt(ev/ne):0, jn?sqrt(jt/jn):0, jn?sqrt(jv/jn):0, jn?sqrt(jc/jn):0);
  printf("  the smoothing's speed: %.1f mm/s at rest (alpha %.2f), %.1f mm/s moving\n", restN?restSpeed/restN:0, restN?restAlpha/restN:0, moveN?moveSpeed/moveN:0);
  printf("  shaft: %.2f deg rms at rest, %.2f deg rms while turning (1 rad/s)\n", nr?sqrt(angRest/nr):0, nt?sqrt(angTurn/nt):0);
  return 0; }
