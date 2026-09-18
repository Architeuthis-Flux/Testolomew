# Testolomew

The test bed for Jumperless V6 ideas: a [nanoCH32H417](https://github.com/wuxx/nanoCH32H417) dev board (WCH CH32H417, the coprocessor planned for V6), PlatformIO, and firmware laid out like [JumperlOS](https://github.com/Architeuthis-Flux/JumperlOS) so that whatever works here can be carried over.

First experiment: **locating a magnet in 3D with a 4×2 array of TMAG5273 Hall sensors**, as a way to track a cable-free probe, and showing it on the board's LCD.

![The 3D view, rendered on the host from simulated sensor readings](docs/magview-simulated.png)

*The LCD view (fixed camera and top-down) of the bench array, rendered on a PC from simulated noisy sensor readings through the real fit and drawing code. The purple ellipse under the magnet is the fix's own 2-σ error bar.*

- `docs/wireless-probe-sensing.md` is everything learned so far about wireless probe sensing: the survey of methods, the numbers, the magnetometer design and its first-run checklist, and the CH32H417 toolchain notes.
- `docs/ratio-ladder-poc-spec.md` is the spec for the other prototype (capacitive ratio ladder on V5 hardware).

## Build and run

```sh
pio run                   # build
pio run -t upload         # flash with wlink through the board's own WCH-LinkE (its own USB-C port, not the chip's)
pio device monitor        # console: the WCH-LinkE's serial port, 115200
pio test -e native        # host-side tests of the magnet fit, no board needed
pio run -t compiledb      # compile_commands.json for clangd
```

The first build downloads the CH32 platform, the RISC-V toolchain and the CH32H4 Arduino core.

The console takes single-character commands; `?` lists them. With every module on:

| Key | |
|---|---|
| `?` | command list |
| `X` | service table (runs and timing per service) |
| `m` | magnetometer array status: addresses, part variants, latest fields |
| `z` | re-zero the ambient baseline (magnet away) |
| `p` | power-cycle and re-address every sensor |
| `i` | identify: print which sensor reads strongest |
| `b` | bus check: are there pull-ups on SCL/SDA, and what acknowledges with each sensor powered alone |
| `f` | stream field frames as CSV |
| `l` | latest probe fix, with its own ± error bar in mm on each axis, and how many sensors see the magnet plainly + faintly |
| `k` / `K` | learn this magnet's strength and hold the fit to it (steadier height) / forget it |
| `t` | where the probe's point is: `t12⏎` = the magnet's centre is 12 mm up the shaft from the point. The magnet is magnetised along the shaft, so its pole direction is the shaft, and the point is that far down it (down = the lower end; a probe is not held upside down). `MAGLOC_TIP_OFFSET_MM` makes it permanent |
| `o` | orientation check: with a magnet held 1–2 cm over the array, which row order / rotation / board side explains it |
| `d` | stream probe fixes as CSV |
| `v` | cycle the 3D view's camera: sway, fixed, spin, top-down |
| `r` | row mode: which breadboard row the probe is over, Jumperless numbering (1–30 along the far half, 31–60 along the near half, 31 facing 1). The row large on the LCD, coloured by how sure the fix is, the breadboard drawn on the board; a line on the console twice a second (row, offset into it, hole 1–5 counted from the channel, ± in rows, % sure) |
| `h` | hold-still test: 2 s of fixes → the row and hole, the scatter in rows and mm against the fit's own error bar, how many single fixes called that row, and how far it moved since the last test. Step one row, `h` again: it should say +1.00 |
| `c` | calibrate the rows: asks for twelve taps (rows 1, 15, 30, 60, 45, 31, each at the hole next to the channel and at the outermost hole) and fits the grid to them: where the breadboard lies, its angle, a scale each way and skew. A tap is the probe resting in the hole, still for a second, at the height the board holds the magnet at (learned from the first tap). The LCD shows which hole is wanted, a ring on it, and a bar that fills while the tap is taken. Prints what each tap still misses its hole by after the fit, which is the accuracy of the whole scheme in mm, and a `ROWCOUNT_GRID_AT_BOOT` line to paste into `RowCounter.h` to keep it. `c` again cancels |
| `R` | one more anchor by hand: `R30⏎` = "the probe is on row 30 now, in the hole next to the channel" (any row 1–60). Every anchor given is kept and the grid is fitted to all of them: one moves the breadboard into place, two or more 10 mm apart also give the angle and the scale |
| `C` | forget the anchors: back to the grid the firmware boots with |

## Wiring

All in `src/board/BoardPins.h`. For the magnetometer array:

| | Pin |
|---|---|
| SCL / SDA | PA8 / PC9 (I2C3), **with external pull-ups to 3.3 V**; this chip has no internal ones for I²C |
| Sensor VCC 0–7 | PD0, PC10, PC11, PA14, PD6, PD2, PD3, PD4 |
| Sensor ground, set A / set B | PA15 / PD5 |

The sensors' supplies are GPIOs because a TMAG5273 forgets an assigned I²C address whenever it loses power: they have to be powered up one at a time and re-addressed at every boot. `src/magarray/MagArray.h` explains the sequence. The array's geometry (pitch, order, each sensor's rotation) is `src/magarray/MagArrayConfig.h`; the default 20 mm pitch is a placeholder to be measured.

The LCD is whatever ST7789 panel is on the board's 12-pin FPC connector; `src/display/ST7789.h` has the numbers for the common ones (default 1.54" 240×240).

## Layout

```
src/
  main.cpp          setup() begins each module and registers its Service; loop() is the scheduler
  config.h          which modules are in the build
  jos/              Service + jOSmanager: JumperlOS's scheduler interface, cut down
  board/            every pin, and board-level helpers
  console/          single-character serial commands; modules register their own
  common/           Vec3
  magarray/         TMAG5273 driver; MagArray service (addressing, sampling, baseline)
  magfit/           MagFit (pure-math dipole fit, host-testable); MagLocator service
  display/          ST7789 framebuffer push; MagView service (the 3D view)
  rowcount/         RowGrid (pure math: position -> breadboard row 1-60 and hole, host-testable); RowCounter service
test/test_magfit/   host tests of the fit
test/test_rowgrid/  host tests of the row grid
tools/              magcal (array self-calibration from a recording) and the recordings
attic/              things tried and dropped, out of the build (the two-magnet probe fit)
scripts/            src_subdirs_include.py, same as JumperlOS: every folder under src/ is on the include path
docs/
```

Conventions are JumperlOS's: Arduino framework, feature folders under `src/` with bare `#include "Foo.h"`, a module is one or more `Service` subclasses with a `getInstance()` singleton and an `extern` reference, `.clang-format` copied from JumperlOS (LLVM base, 4 spaces, no column limit), plain structs and free functions for drivers, no templates or exceptions. `src/jos/JumperlOS.h` deliberately has the same file name and the same `Service` interface as JumperlOS, so a module's scheduling code moves over unchanged.

## Calibrating the array

`tools/magcal.cpp` works out where each sensor really is, how it is turned, and its gain, from a recording of a magnet being waved over the array: stream frames with `f`, save them to a file, measure one real distance (sensor 4 to sensor 7) for scale, and

```sh
cd tools
c++ -std=c++11 -O2 -I../src/magfit -I../src/common magcal.cpp ../src/magfit/MagFit.cpp -o magcal
./magcal recordings/my-recording.txt 53.4 100     # 53.4 = mm from sensor 4 to 7; frames after 100 s are held back as a test
```

Copy the table it prints into `src/magarray/MagArrayConfig.h` (and into `magcal.cpp`'s own copy, which must match the firmware the recording was made with). `tools/recordings/` has the recording the current table came from and the ruler test that checked it. Before calibrating, `i` (sensor order) and `o` (rotation, side, row order) get the table close enough for the solver to converge.

## Adding a module

1. Make a folder under `src/` and write a `Service` in it. `src/magfit/MagLocator.*` is the smallest complete example: `begin()`, `service()`, `getName()`, `getPriority()`, `periodUs()`.
2. Take pins from `BoardPins.h` (add them there), not from literals.
3. Register console commands from the module's own `begin()` with `consoleAddCommand()`.
4. Add a `MODULE_…` flag to `src/config.h` and a three-line block to `setup()` in `src/main.cpp`.

Removing one is the reverse. Setting its flag to 0 stops it being started or scheduled; its files still compile, so a switched-off experiment cannot quietly rot.

Both cores are available through the Arduino core (`setup1()`/`loop1()` run on the 100 MHz V3F, like arduino-pico's second core). Everything currently runs on the 400 MHz V5F.

## Toolchain

`platform-ch32v` (pinned commit) with the `ArduinoCore-CH32H4` Arduino core, board `ch32h417qeu6_evt_r0` (WCH's EVT board, same chip). The core raises the chip's VIO18 I/O rail from its 1.2 V reset value to 3.3 V at boot, which almost every pin on this board depends on. `docs/wireless-probe-sensing.md` §7 has the notes on the alternatives that were looked at (WCH's SDK via `noneos-sdk`, ch32fun).
