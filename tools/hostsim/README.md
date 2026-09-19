# Host simulation of the whole firmware

The firmware's modules run on a PC against a stub of the Arduino API (`Arduino.h`, `Print.h`, `Wire.h`, `SPI.h` here), with the sensor readings made from a simulated magnet. Two programs:

- `screens.cpp` moves a probe magnet over the bench array through `MagLocator` (fit + tracker), `RowCounter`, `ProbeLeds`, `Input`, `Ui` and `MagView`, types keys "on the console", and writes a PNG of every screen (`s1_… .png`); `docs/screens-simulated.png` is a montage of them. It also runs the settings module against the RAM `EEPROM.h` and prints two PASS/FAIL lines: the boot-time load must not switch row mode or the LED chain (the modes it does not save), and a changed value (`S23`) is written once after the settle time and not again for the same value.
- `soak.cpp` is ten minutes of a random hand (random goals, 3 m/s² accelerations, a 0.3 mT glitch on one axis every 40 frames, a 140 ms dropout every 10 s) through the same chain, looking for NaNs, stuck states and tracking error.

Build from this folder with the Adafruit GFX and BusIO libraries checked out somewhere (`screens.cpp` draws into the real `GFXcanvas16`; `soak.cpp` does not need them):

```sh
S=../../src; G=/path/to/Adafruit-GFX-Library; B=/path/to/Adafruit_BusIO
g++ -std=c++11 -O1 -w -DARDUINO=10808 -I . -I $S -I $S/common -I $S/jos -I $S/console -I $S/magarray -I $S/magfit -I $S/rowcount -I $S/display -I $S/probeled -I $S/ui -I $S/play -I $S/settings -I $S/board -I $G -I $B \
    screens.cpp LedStripStub.cpp MagSamplerStub.cpp $S/magarray/MagArray.cpp $S/magarray/TMAG5273.cpp $S/magfit/MagFit.cpp $S/magfit/MagTracker.cpp $S/magfit/MagLocator.cpp \
    $S/rowcount/RowGrid.cpp $S/rowcount/RowCounter.cpp $S/display/MagView.cpp $S/display/FastDraw.cpp $S/probeled/ProbeLeds.cpp $S/probeled/ProbeLedService.cpp $S/play/Play.cpp $S/settings/Settings.cpp \
    $S/ui/Ui.cpp $S/ui/Input.cpp $S/ui/UiStream.cpp $S/ui/Menu.cpp $S/ui/Camera.cpp $S/console/Console.cpp $S/jos/JumperlOS.cpp $G/Adafruit_GFX.cpp -o screens && ./screens

g++ -std=c++11 -O2 -w -I . -I $S -I $S/common -I $S/jos -I $S/console -I $S/magarray -I $S/magfit -I $S/rowcount -I $S/probeled -I $S/play -I $S/board \
    soak.cpp LedStripStub.cpp MagSamplerStub.cpp $S/magarray/MagArray.cpp $S/magarray/TMAG5273.cpp $S/magfit/MagFit.cpp $S/magfit/MagTracker.cpp $S/magfit/MagLocator.cpp \
    $S/rowcount/RowGrid.cpp $S/rowcount/RowCounter.cpp $S/probeled/ProbeLeds.cpp $S/probeled/ProbeLedService.cpp $S/play/Play.cpp $S/console/Console.cpp $S/jos/JumperlOS.cpp -o soak && ./soak

g++ -std=c++11 -O2 -w -I . -I $S -I $S/common -I $S/jos -I $S/console -I $S/magarray -I $S/magfit -I $S/rowcount -I $S/probeled -I $S/board \
    pencil.cpp LedStripStub.cpp MagSamplerStub.cpp $S/magarray/MagArray.cpp $S/magarray/TMAG5273.cpp $S/magfit/MagFit.cpp $S/magfit/MagTracker.cpp $S/magfit/MagLocator.cpp \
    $S/console/Console.cpp $S/jos/JumperlOS.cpp -o pencil && ./pencil [viewHz viewBeta cursorHz cursorBeta shaftHz shaftBeta accel]
```

`pencil.cpp` is the benchmark the smoothing levers were set by: a hand that writes (50-250 mm/s strokes with pauses, the pencil turning at up to 1 rad/s) through the real locator and tracker; it reports each output's error while moving and the delay of the truth it best matches (its lag), the frame-to-frame jitter at rest, and the shaft's angle error at rest and while turning. Run it with the six levers (and the tracker's process noise, and the smoothing's jitter allowance `MAGLOC_SPEED_JITTER_K` as an eighth number) to see what a change buys; a last argument `far` has the hand hovering 12-22 mm above the surface instead of writing on it, which is where the weak-field smoothing and its speed rule matter.

`navtest.cpp` drives the nav stick's contacts through `Input` with staggered closings and openings (the ALPS RKJXM1015004's push is a contact of its own that also closes on every tilt) and checks that a press comes out as a press and a tilt as a direction, never the other:

```sh
g++ -std=c++11 -O1 -w -I . -I $S -I $S/common -I $S/jos -I $S/console -I $S/ui -I $S/board navtest.cpp $S/ui/Input.cpp $S/console/Console.cpp $S/jos/JumperlOS.cpp -o navtest && ./navtest
```

`LedStripStub.cpp` stands in for the WS2812 driver (no SPI on a PC: the service reports no strip and renders for the preview), `MagSamplerStub.cpp` for the other core's sampler (the array reads its simulated bus itself), and `EEPROM.h` for the core's EEPROM library (a RAM mirror). The stub defines `private` as `public` for the includes so the programs can poke module internals; it is a test rig, not an example of how to use the modules. `ppm2png.py` turns the PPM frames into PNGs (3× upscaled). These were written in the scratch space on 2026-09-17/18 and copied here because they are the only test of the whole chain; the soak's numbers are quoted in `docs/wireless-probe-sensing.md`.
