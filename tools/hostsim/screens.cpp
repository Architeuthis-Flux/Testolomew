// The whole firmware on the host: a single probe magnet moved over the bench
// array, through MagLocator (fit + tracker), RowCounter, ProbeLeds, Input, Ui
// and MagView; keys typed "on the console"; frames of every screen written
// as PNGs. Not part of the project.
#define private public
#include "MagArray.h"
#include "MagLocator.h"
#include "MagView.h"
#include "RowCounter.h"
#include "ProbeLedService.h"
#include "Ui.h"
#include "Input.h"
#include "UiStream.h"
#undef private
#include <Adafruit_GFX.h>
#include "ST7789.h"
#include "Console.h"
#include "EEPROM.h"
#include "Settings.h"
#include "Play.h"
EEPROMClass EEPROM;
uint32_t simMillis = 0; int simPinLevel[64] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// A serial port whose input the test types.
class FakeSerial : public Stream {
  public:
    char q[256]; int head = 0, count = 0;
    int available() override { return count; }
    int read() override { if (!count) return -1; char c = q[head]; head = (head + 1) % 256; count--; return c; }
    int peek() override { return count ? q[head] : -1; }
    void type(const char* s) { while (*s) { q[(head + count) % 256] = *s++; count++; } }
};
FakeSerial fake;
Stream Serial; // unused by the console (it talks through uiStream -> fake)
TwoWire Wire;
bool boardVioIs3V3(void) { return true; }
void boardLedsInit(void) {}
void boardLed(int, bool) {}
bool st7789Begin(void) { return true; }
void st7789PushRows(uint16_t*, int, int) {}
bool st7789PushRowsStart(uint16_t*, int, int) { return true; }
bool st7789PushBusy(void) { return false; }
void st7789PushFinish(void) {}

static float gauss(float s){ float u1=(rand()+1.0f)/((float)RAND_MAX+2.0f), u2=(rand()+1.0f)/((float)RAND_MAX+2.0f); return s*sqrtf(-2*logf(u1))*cosf(2*M_PI*u2);}
static void writePpm(const char* name, const uint16_t* fb) {
    FILE* f = fopen(name, "wb"); fprintf(f, "P6\n%d %d\n255\n", LCD_WIDTH, LCD_HEIGHT);
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) { uint16_t c = fb[i]; uint8_t rgb[3] = { (uint8_t)(((c >> 11) & 31) * 255 / 31), (uint8_t)(((c >> 5) & 63) * 255 / 63), (uint8_t)((c & 31) * 255 / 31) }; fwrite(rgb, 1, 3, f); }
    fclose(f);
}
static void snap(const char* name) {
    magView.pushRow = -1; magView.drawn = false; magView.service(); // draws a frame and starts its (stubbed) push: it is now `shown`
    char ppm[64]; snprintf(ppm, sizeof ppm, "%s.ppm", name); writePpm(ppm, magView.shown->getBuffer());
    char cmd[200]; snprintf(cmd, sizeof cmd, "python3 ppm2png.py %s.ppm %s.png", name, name); system(cmd);
    printf("wrote %s.png\n", name);
}
// One 10 ms frame of the world: the magnet at `magnet`, magnetised along `shaft`; dropout = no readings.
static void frame(Vec3 magnet, Vec3 shaft, bool dropout = false) {
    simMillis += 10;
    Vec3 moment = { 4200 * shaft.x, 4200 * shaft.y, 4200 * shaft.z };
    for (int i = 0; i < MAG_SENSOR_COUNT; i++) {
        magArray.field[i] = magFitDipoleField(magArray.position[i], magnet, moment);
        magArray.field[i].x += gauss(0.005f); magArray.field[i].y += gauss(0.005f); magArray.field[i].z += gauss(0.005f);
        magArray.fresh[i] = !dropout;
    }
    magArray.frameCount++;
    magLocator.service(); rowCounter.service(); probeLeds.lastUs = 0; probeLeds.service();
    console.service(); input.service(); ui.service(); settings.service(); play.service();
    // the display runs at ~30 fps: advance the camera every third frame
    if ((simMillis / 10) % 3 == 0) magView.aimCamera(simMillis);
}
static void type(const char* keys, Vec3 magnet, Vec3 shaft) { fake.type(keys); int n = 8 + 12 * (int)strlen(keys); for (int k = 0; k < n; k++) frame(magnet, shaft); }

int main() {
    uiStream.begin(&fake); console.begin(&uiStream); magArray.begin();
    for (int i = 0; i < MAG_SENSOR_COUNT; i++) magArray.sensors[i].ok = true;
    magArray.baselineLeft = 0; magArray.baselineCount = 1;
    magLocator.begin(); rowCounter.begin(); probeLeds.begin(); magView.begin(); input.begin(); ui.begin();
    bool rowModeBefore = rowCounter.active, stripBefore = probeLeds.strip;
    settings.begin(&ui.menu); ui.settingsLoaded(); // as main.cpp: the (empty) saved settings in, the toggle/choice items into the modules
    printf("settings loaded: row mode %s -> %s, chain %s -> %s (%s: the modes must not be switched by the load)\n", rowModeBefore ? "on" : "off", rowCounter.active ? "on" : "off",
           stripBefore ? "on" : "off", probeLeds.strip ? "on" : "off", rowModeBefore == rowCounter.active && stripBefore == probeLeds.strip ? "PASS" : "FAIL");
    srand(4);
    console.printHelp();

    // The breadboard as the firmware boots with it; row 14 hole 3, top half, point down (z 17.5), shaft leaning 20 deg toward +x.
    float lean = 20 * 0.01745f; Vec3 shaft = { sinf(lean), 0, cosf(lean) };
    Vec3 onRow14 = rowGridToBoard(&rowCounter.grid, 14.0f, 3.81f + 2 * 2.54f); onRow14.z = 17.5f;
    printf("row 14 hole 3 is at x %.1f y %.1f\n", onRow14.x, onRow14.y);
    fake.type("r"); // row mode on
    // The surface learned from where the point bottoms out: set too high, it comes down to the resting height within a few seconds of touching.
    magLocator.boardZ = 25.0f;
    for (int k = 0; k < 400; k++) frame(onRow14, shaft);
    printf("surface: set 25.0, the point rests at 17.5, learned %.1f (%s)\n", magLocator.boardZ, fabsf(magLocator.boardZ - 17.5f) < 0.8f ? "PASS" : "FAIL");
    for (int k = 0; k < 150; k++) frame(onRow14, shaft);
    magLocator.printFix(&uiStream); rowCounter.printReading(&uiStream);
    snap("s1_scene_on_row");

    // Lift 10 mm and aim: the pointer reaches down the shaft to the surface.
    Vec3 lifted = onRow14; lifted.z = 27.5f;
    for (int k = 0; k < 120; k++) frame(lifted, shaft);
    magLocator.printFix(&uiStream); rowCounter.printReading(&uiStream);
    { const MagProbeFix& f = magLocator.fix; const MagTrack& t = magLocator.track;
      printf("DBG truth %.2f %.2f %.2f | raw %.2f %.2f %.2f axis %.3f %.3f %.3f shaft %.3f %.3f %.3f | rawPointer %.2f %.2f %.2f boardZ %.1f tipOff %.1f angle %.0f | cursor %.2f %.2f reach %.2f\n",
        lifted.x, lifted.y, lifted.z, f.rawMagnet.x, f.rawMagnet.y, f.rawMagnet.z, f.axis.x, f.axis.y, f.axis.z, f.shaft.x, f.shaft.y, f.shaft.z, f.rawPointer.x, f.rawPointer.y, f.rawPointer.z, magLocator.boardZ, magLocator.tipOffsetMm, magLocator.magnetAngleDeg, t.cursor.x, t.cursor.y, t.reachMm); }
    snap("s2_scene_lifted_pointed");
    type("e", lifted, shaft); // LED screen
    snap("s3_leds_lifted");
    // Paint mode on the draw screen: the joystick moves the colour wheel's marker (typed , . ; ' nudge it), / toggles erase,
    // the nav stick's up/down set the paint brightness and right/left the brush. A stroke on the board paints LEDs.
    type("e", lifted, shaft); type("e", lifted, shaft); // log, then draw (which is the paint app: the play service switches to paint by itself)
    for (int k = 0; k < 5; k++) frame(lifted, shaft);
    printf("draw screen: play mode %s (expected paint)\n", playModeNames[play.mode]);
    input.joystickFitted = false; // the typed joystick keys, not the (unwired here) analog pins
    for (int k = 0; k < 8; k++) type(".", lifted, shaft);  // right along the wheel...
    for (int k = 0; k < 5; k++) type("'", lifted, shaft);  // ...and up: a hue between red and yellow, near the rim
    type("\x1b[A", lifted, shaft); type("\x1b[A", lifted, shaft); // brightness up twice
    type("\x1b[C", lifted, shaft);                                 // brush 1
    Vec3 stroke = onRow14; stroke.z = 17.5f;
    for (int k = 0; k < 80; k++) { stroke.x += 0.25f; frame(stroke, shaft); }
    printf("paint: hue %.0f sat %.2f bright %.2f brush %.0f erase %d, %d LEDs painted\n", play.paintHue, play.paintSat, play.paintBright, play.brushSize, play.erase, [&]{ int n = 0; for (int i = 0; i < PROBELED_MAX; i++) n += play.paint.level[i] > 0; return n; }());
    snap("s3b_draw_paint");
    type("/", lifted, shaft); // erase
    snap("s3c_draw_erase");
    type("/", lifted, shaft);
    type("\x1b[C", lifted, shaft); // brush 2: the ring
    { int n = 0; for (int i = 0; i < PROBELED_MAX; i++) n += play.paint.level[i] > 0; printf("before leaving: erase %d, %d painted\n", play.erase, n); }
    type("e", stroke, shaft); type("e", stroke, shaft); // scene, then the LED screen: the brush ring as the cursor over the paint
    for (int k = 0; k < 20; k++) frame(stroke, shaft);
    snap("s3d_leds_brush");
    for (int k = 0; k < 5; k++) frame(lifted, shaft);
    int kept = 0; for (int i = 0; i < PROBELED_MAX; i++) kept += play.paint.level[i] > 0;
    printf("off the draw screen: play mode %s (expected off), %d painted LEDs kept\n", playModeNames[play.mode], kept);
    play.clearPaint();
    for (int k = 0; k < 20; k++) frame(lifted, shaft);
    type("u", lifted, shaft); // cursor under the tip
    for (int k = 0; k < 30; k++) frame(lifted, shaft);
    snap("s4_leds_under");
    type("u", lifted, shaft);

    // A 200 ms dropout while moving along the rows: coasting.
    Vec3 moving = lifted;
    for (int k = 0; k < 60; k++) { moving.x += 0.3f; frame(moving, shaft); }
    type("e", moving, shaft); type("e", moving, shaft); type("e", moving, shaft); // back to the scene (through the log and draw screens)
    for (int k = 0; k < 15; k++) { moving.x += 0.3f; frame(moving, shaft, true); }
    snap("s5_scene_coasting");
    for (int k = 0; k < 30; k++) frame(moving, shaft);

    // Far away: rough.
    Vec3 far = { 27, 22, 54 };
    for (int k = 0; k < 120; k++) frame(far, shaft);
    magLocator.printFix(&uiStream);
    snap("s6_scene_rough");
    type("e", far, shaft);
    snap("s7_leds_rough");
    type("e", far, shaft); type("e", far, shaft); type("e", far, shaft);

    // The menu: open, into the tracker page, then the commands page, run l.
    for (int k = 0; k < 60; k++) frame(onRow14, shaft);
    type("\t", onRow14, shaft);
    snap("s8_menu_root");
    type("\r", onRow14, shaft);
    snap("s9_menu_tracker");
    type("`", onRow14, shaft); // back
    // Down to the commands page and, in it, to `l` - by label, so new pages and commands do not move the target.
    while (strcmp(ui.menu.items[menuVisibleItem(&ui.menu, ui.menu.cursor)].label, "commands") != 0) type("\x1b[B", onRow14, shaft);
    type("\r", onRow14, shaft);
    snap("s10_menu_commands");
    while (ui.menu.items[menuVisibleItem(&ui.menu, ui.menu.cursor)].label[0] != 'l') type("\x1b[B", onRow14, shaft);
    type("\r", onRow14, shaft); // run it: the log screen
    for (int k = 0; k < 10; k++) frame(onRow14, shaft);
    snap("s11_log_after_l");
    type("\t", onRow14, shaft); // close the menu
    type("e", onRow14, shaft); type("e", onRow14, shaft); // scene (through the draw screen)

    // POV camera, probe tilted 25 deg.
    float lean2 = 25 * 0.01745f; Vec3 shaft2 = { sinf(lean2) * 0.7f, sinf(lean2) * 0.7f, cosf(lean2) };
    type("vvvvv", lifted, shaft2); // fixed -> sway -> spin -> top -> follow -> POV
    for (int k = 0; k < 120; k++) frame(lifted, shaft2);
    printf("camera mode %s yaw %.0f el %.0f dist %.0f zoom %.1f target %.0f %.0f %.0f\n", cameraModeNames[magView.cam.mode], magView.cam.yawDeg, magView.cam.elevationDeg, magView.cam.distance, magView.cam.zoom, magView.cam.target.x, magView.cam.target.y, magView.cam.target.z);
    snap("s12_scene_pov");
    type("v", lifted, shaft2); // fixed again
    type("v", lifted, shaft2); type("v", lifted, shaft2); type("v", lifted, shaft2); // top
    for (int k = 0; k < 120; k++) frame(lifted, shaft2);
    snap("s13_scene_top");
    // orbit with the joystick and pan with the nav keys
    type("v", lifted, shaft2); type("v", lifted, shaft2); type("v", lifted, shaft2); // fixed
    for (int k = 0; k < 20; k++) type(".", lifted, shaft2);
    for (int k = 0; k < 6; k++) type("\x1b[C", lifted, shaft2);
    for (int k = 0; k < 60; k++) frame(lifted, shaft2);
    printf("after orbit/pan: yaw %.0f target %.0f %.0f\n", magView.cam.yawDeg, magView.cam.target.x, magView.cam.target.y);
    snap("s14_scene_orbited");
    printf("track: %lu accepted %lu dropped %lu reinits\n", (unsigned long)magLocator.track.accepted, (unsigned long)magLocator.track.dropped, (unsigned long)magLocator.track.reinits);

    // The settings: a console command that changes a saved value (S: the
    // surface height) is written once it has stood for the settle time - the
    // serialisation being done a few records a run - and a second S the same
    // is not written again. Then `s` shows it.
    for (int k = 0; k < 400; k++) frame(onRow14, shaft2); // whatever the run above left unsettled goes in first
    uint32_t savesBefore = settings.saves; int commitsBefore = EEPROM.commits;
    type("S23\r", onRow14, shaft2);
    for (int k = 0; k < 150; k++) frame(onRow14, shaft2); // 1.5 s: not yet
    bool early = settings.saves == savesBefore;
    for (int k = 0; k < 250; k++) frame(onRow14, shaft2); // 4 s in all: written
    bool written = settings.saves == savesBefore + 1 && EEPROM.commits > commitsBefore;
    type("S23\r", onRow14, shaft2);
    for (int k = 0; k < 400; k++) frame(onRow14, shaft2);
    bool once = settings.saves == savesBefore + 1;
    type("s", onRow14, shaft2);
    printf("settings: %s (not before the settle time: %s; written after it: %s, saves %lu commits %d; the same value again not written: %s)\n",
           early && written && once ? "PASS" : "FAIL", early ? "yes" : "NO", written ? "yes" : "NO", (unsigned long)settings.saves, EEPROM.commits, once ? "yes" : "NO");
    return 0;
}
