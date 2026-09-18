# Testolomew

Test bed for Jumperless V6 ideas on a wuxx nanoCH32H417 (CH32H417QEU6). The design context for V6 lives in `~/Desktop/V6pile` (start with its `CLAUDE.md`); the firmware this is meant to feed is `~/Documents/GitHub/JumperlOS`.

## Commands

- `pio run` builds; `pio run -t upload` flashes with wlink through the onboard WCH-LinkE (OpenOCD's upload crashes on reset with this chip, so `upload_protocol = wlink`); console is the WCH-LinkE serial port at 115200.
- `pio test -e native` runs the host-side tests (pure-math code only, currently `src/magfit/MagFit.cpp` and `src/rowcount/RowGrid.cpp`).
- `pio run -t compiledb` regenerates `compile_commands.json` for clangd.

## Rules of the house

- Match JumperlOS: Arduino framework, feature folders under `src/` with bare `#include "Foo.h"` (every folder is on the include path via `scripts/src_subdirs_include.py`), modules are `Service` subclasses registered with `jOS` in `main.cpp`, `.clang-format` is JumperlOS's (4 spaces, spaces inside parentheses, no column limit), SPDX header on every file.
- Keep the C++ plain: structs, free functions for drivers, one singleton class per service. No templates, exceptions, STL containers or inheritance beyond `Service`.
- `src/jos/JumperlOS.h` mirrors the `Service`/`jOSmanager` interface of JumperlOS's `src/JumperlOS.h`. Do not add scheduler features here that JumperlOS lacks; the point is that modules port over unchanged.
- Every pin comes from `src/board/BoardPins.h`. Modules register their own console commands with `consoleAddCommand()`; `Console.cpp` knows no module.
- Pure math that can run on the host goes in its own `.cpp` with no Arduino includes and gets a test under `test/`.
- Values that are guesses are marked `ASSUMPTION` in the source (sensor pitch, the PA14 power pin, the probe tip offset). Replace them with measurements; do not build on them.

## Hardware facts that bite

- Only PA5–PA7 and PE2–PE6 are fixed 3.3 V pins. Everything else is on the VIO18 rail, which resets to 1.2 V; the Arduino core sets it to 3.3 V at boot (`boardVioIs3V3()` checks). Any other framework must set `PWR->CTLR[12:10] = 3` first.
- I²C has no internal pull-ups on this chip. SCL/SDA pairs are fixed by the silicon (PA8 goes with PC9).
- TMAG5273 I²C addresses are volatile: reassign after every power-up, one sensor powered at a time. Its sign convention is positive for field going *into* the package top (see `MagArray::toBoardFrame`).
- The platform is pinned to a commit in `platformio.ini`; the CH32H4 Arduino core is days old, so bump deliberately.

## Docs

`docs/wireless-probe-sensing.md` is the consolidated write-up of the wireless-probe work (survey, ratio ladder, magnetometer array, first-run checklist, toolchain notes). Update it when a bench result replaces an estimate.
