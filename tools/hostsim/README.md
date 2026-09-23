# The firmware on the host

The firmware's modules run on a PC against stubs of what the chip provides (`Arduino.h`, `Print.h`, `Wire.h`, `SPI.h`, `EEPROM.h`, and the LCD, LED strip and other-core sampler in `*Stub.cpp`). `make` builds everything here; GFX comes from PlatformIO's libdeps, so run `pio run` once first.

## The simulator: `make sim`, `./sim <script> [-q]`

`sim.cpp` boots the firmware through main.cpp's own `setup()` and then runs the real scheduler (`jOS.serviceAll()`) with the clock advancing 250 µs a pass, so every service ticks at its own period as on the board. A `WorldService` writes the dipole field of a simulated magnet into the array's frames (`MagArray::useSimulatedFrames()` tells the array the frames come from outside). The script is typed on the console exactly as on the board - letters, keys and the `:verbs` (`:help` lists them) - with a few directives for the world and for checks:

| line | what it does |
|---|---|
| `:screen`, `:probe row 14 3`, `:screen:ascii` ... | any verb: typed with Enter; the run goes on until its frame closes (or 5 s) |
| `e`, `\t`, `\e[A`, `` ` ``, `/`, `S23\r` | typed keys (escapes `\e \r \n \t`), then a moment for them to play |
| `@run <ms>` | let the loop run |
| `@magnet <x> <y> <z> <sx> <sy> <sz>` | the magnet's centre (mm) and axis |
| `@magnet row <r> <h> [up mm] [lean deg]` | the probe's point in that hole, leaning toward +x |
| `@magnet off` | no magnet |
| `@move <dx> <dy> <dz> <ms>` | glide the magnet by that much over that long |
| `@dropout <ms>` | no readings for that long |
| `@bias <i> <x> <y> <z>` | a zero error at sensor i, mT board frame, on every frame |
| `@gain <i> <f>` | sensor i reads this much of the true field |
| `@dead <i> <ms>` | sensor i not read for that long |
| `@strength <n>` | the magnet's moment, mT*mm^3 (the locator's held value is its belief) |
| `@tip <mm>`, `@surface <mm>` | where the magnet really sits up the shaft, where the board's top really is |
| `@baselines` | print each sensor's zero and field |
| `@noise <x> <y> <z>` | a TMAG5273's noise a frame per axis, mT (the other types by their ratio); the default 0.010 isotropic, the bench 0.012 0.012 0.006 |
| `@place <i> <dx> <dy> <dz>` | sensor i really sits this far (mm) from where the table says |
| `@strip on\|off` | pretend a real LED chain is wired (the display's LED-first rule) |
| `@type "text" [n] [gap ms]` | type text n times with a gap (a run of joystick nudges) |
| `@expect "text" [ms]` | the text must appear in what the firmware printed for the previous script line (waiting up to ms) |
| `@seen "text"` | the text must have appeared anywhere since the script began (a line whose moment is not known) |
| `@seed <n>`, `@echo <text>`, `@quit` | |

Two verbs exist only in the simulator: `:screen:png <file>` writes what the panel shows as a PNG, and `:screen:verify` checks that the dump path the board uses (`:screen:dump` → `MagView::copyShownRow`, which un-swaps the bytes the LCD push swapped in place) gives exactly what the panel received. The exit code is 1 if any `@expect` failed; the boot line checks that loading the settings did not switch the modes and that the menu table has room.

- `make check` runs `scenes/check.txt`, the regression set: the verbs, a simulated probe seen by the tracker, the row counter and the LEDs, the dumps, the menu driven from the console, the settings written after they settle with every page's keys present.
- `make screens` runs `scenes/screens.txt` (the probe on a row and lifted, the LED preview, paint and erase, coasting, a far probe, the menu pages, the log, the POV, top and orbited cameras) into `out/*.png` and tiles them into `docs/screens-simulated.png` with `montage.py`.

## The benches

- `make soak`: ten minutes of a random hand (random goals, 3 m/s² accelerations, a 0.3 mT glitch every 40 frames, a 140 ms dropout every 10 s) through the locator, tracker, row counter and LEDs, looking for NaNs, stuck states and tracking error. Its numbers are quoted in `docs/wireless-probe-sensing.md`.
- `make pencil`: the benchmark the smoothing levers were set by - a hand that writes (50-250 mm/s strokes with pauses, the pencil turning) through the real locator and tracker; reports each output's error and lag, the jitter at rest, the shaft's angle error. `./pencil [viewHz viewBeta cursorHz cursorBeta shaftHz shaftBeta accel [jitterK]] [far] [steady N]` (`steady N`: the fit on a steady budget of N iterations a frame, MagLocator's "fit load").
- `make navtest`: the nav stick decoder with staggered contact closings and openings (the ALPS stick's push contact closes on every tilt too): a press must come out as a press and a tilt as a direction, never the other.

## Tools

- `../screendump.py <port> out.png [--b64] [--step N]` grabs the board's screen over the console (`:screen:dump`) into a PNG; `--ascii` grabs `:screen:ascii` as text; `--decode captured.txt out.png` decodes a dump captured some other way. `../rgb565png.py` is the decoder and PNG writer both it and `montage.py` use.
- `montage.py out.png a.png b.png ...` tiles PNGs.
- `ppm2png.py` is kept for the old PPM frames.

The old `screens.cpp` (the first host render test, with its own frame loop and `#define private public`) is in `attic/hostsim-screens.cpp`.
