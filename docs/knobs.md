# Testolomew knobs: every setting, what it does, and where it lives

Written 2026-09-19 (midday) as the hand-over for a new chat, after the night of 2026-09-18/19 and the morning's paint work. The firmware in `~/Documents/GitHub/Testolomew` is uncommitted (nothing has been committed since the initial commit; `git status` shows the whole night). The story of *why* each thing is the way it is lives in `docs/wireless-probe-sensing.md` (the "night of 2026-09-18/19" entry and the timed entries after it) and `docs/morning-report-2026-09-19.md`; this file is the reference card.

## 1. How settings work

- **Every NUMBER, TOGGLE and CHOICE item on the on-screen menu is saved** to the flash tail by `src/settings` (the core's EEPROM library), keyed by `page/label`, and put back at boot. Change it in the menu or with a console command; two seconds after it stops changing it is written. `s` prints what is saved, `Z` (menu: "reset settings") puts everything back to the compile-time defaults and forgets the row anchors.
- **Not saved** (modes, in `unsaved[]` in `Settings.cpp`): row mode, the LED chain on/off, the V5 console stream, the play mode. They come up as the firmware boots them: row mode on, chain on, stream off, play off. The settings module captures every item's default from its getter after the modules have begun, and a reset writes through the setters only what differs, so neither a load nor a reset forces a mode (for a day every boot had switched row mode and the chain off through the old mirror variables).
- **Renaming an item retires its saved value** (the old key is dropped, the default takes). `SETTINGS_TUNING_VERSION` (3) does the same for the tracker and smoothing pages and `play/touch` when their defaults improve.
- Also saved, not menu items: the array's **zero** (eight `zero=` lines) and the last good **magnet** fix (for `Y`). The row **anchors** too (none saved at the moment).
- A chip **erase** (not a flash) wipes all of it. Then the board boots with the compiled-in zero `MAG_ZERO_AT_BOOT` (this morning's clean one) rather than zeroing blind - a blind zero with the probe near the board eats its field and every fit after it is wrong (07:36 this morning: misfit 20-50 %, height 5 mm out, LEDs white).

## 2. Home, the apps and the Settings menu

The screen is one of nine things from the **Home** grid (button A): **View** (the 3D scene), **LEDs** (the breadboard's LEDs as the cursor lights them), **Terminal** (the console's log, 40 x 28), **Draw** (the paint app: going to it starts painting, leaving stops), **Settings** (the centre cell: the pages below, over the app), **Target** (the target game, likewise), **Rows** (the counted row, large, with how sure), **Calibrate** (the twelve taps; leaving cancels), **Info** (the build, the sensors, the frame rate, the service table). The probe, the LEDs and the paint go on underneath whatever is open.

The controls mean the same everywhere (the shell, `src/ui/UiShell.cpp`, is the one owner of them):

| control | in an app | Home | a Settings page | Confirm / Result |
|---|---|---|---|---|
| nav / joystick up-down | the app's own | move | move the cursor (wraps) | scroll the result |
| nav / joystick left-right | the app's own | move | **change the value in place**: a toggle flips, a choice cycles, a number steps (x10 after eight repeats), a number-taking action steps its argument | - |
| analog joystick | the app's own (once centred after a change of focus) | - | - | - |
| nav press / joystick press (on the way down) | the app's own | select | enter the page / run the action / flip / cycle | yes / dismiss |
| **A** click (press, in the panes) | Home | select | select | yes / dismiss |
| **A** hold | Settings | - | - | - |
| **B** click | the previous app, else Home | close | back a page (the parent's cursor kept); at the root: close | no / dismiss |
| **B** hold | Home | - | close everything, Home | cancel, Home |
| 20 s idle | - | close (cursor kept) | close (page and cursor kept) | cancel |

A and B never reach an app. A control still held when a pane closes (a nav-left held while B backs out of the menu) is swallowed until it is released, so it never goes on into the app underneath; the joystick is the app's again only once it has been centred. An action run from a Settings page (a console command) shows its output in a **Result** panel over the page - the app underneath never changes - and the destructive ones (`reset settings`, `forget anchors`, `forget strength`, `re-zero`, `power-cycle`) ask first. There is no edit mode: what is shown is what is set, and the settings module writes it two seconds after it stops changing.

From the console the same is driven with `:ui open|menu|close|back|enter|up|down|left|right|go <label>`, `:key <control> [tap|down|up|hold]` and `:joy`; `:screen` prints the app, the panes, the menu page with its items and values, and the probe/play/camera state; `:screen:ascii` and `tools/screendump.py` show the picture.

The pages, item by item. Items marked *(saved)* persist; *(mode)* does not. Every label is the settings module's key, so it stays as it is.

### tracker
| item | what it does | default |
|---|---|---|
| state / fix | info: the track's state and the raw fix | - |
| tracker on | the Kalman tracker between the fits and everything else; off = raw fixes (for comparing) | on |
| floor | `sigmaFloorMm` - the array's systematic error added to every fix's error bar (mm); the tracker's gate and weights use it | 0.3 |
| gate | fixes further than this many sigmas from the track are dropped (a glitch) rather than followed | 4.0 |
| presence | `presentMt` - the strongest smoothed reading (mT) above which a magnet is "there"; hysteresis to half of it | 0.04 |
| far hold | how long a ROUGH track (the far glow) outlives its last rough fix (s) | 2.5 |

### smoothing
| item | what it does | default |
|---|---|---|
| view Hz / view beta | the 1-Euro filter on what the 3D SCENE draws of the magnet: cutoff at rest (Hz), and how fast it opens with speed | 1.5 / 0.5 |
| cursor Hz / cursor beta | the same for the LED cursor / pointer | 1.0 / 0.5 |
| shaft Hz / shaft beta | the same for the shaft direction (the angle the pointer projects along) | 1.0 / 10 |
| camera | the camera's glide time constant (s) for every change of view | 0.18 |
| POV turn / POV move | in POV mode, how slowly the direction follows the shaft and the position the point (s) | 0.6 / 0.3 |
| accel | the tracker's process noise (mm/s²): how jerky a hand may be. 3000 lagged 20 ms; 10000 lags 10 | 10000 |

All of these were set on the bench (`tools/hostsim/pencil.cpp`): a hand writing at 50-250 mm/s through the real locator and tracker, scored on lag and rest jitter.

### cursor
| item | what it does | default |
|---|---|---|
| cursor | `under` = straight under the tip; `aim` = where the tip points on the surface, down the shaft | aim |
| surface | `boardZ` - the breadboard's surface height above the sensors (mm). **Also learned**: when the point bottoms out at the same height three seconds running, below the setting, the setting comes down to it (never up). `S` sets it by hand | 17.5 (compile-time); learned ~10.8 on the bench |
| tip | `tipOffsetMm` - the magnet's centre this far up the shaft from the point (mm); `t` on the console | 0 |
| reach | the pointer never reaches further from the tip than this (mm) | 40 |

### camera
`mode` fixed / sway / spin / top / follow / POV (also `v`), `reset view`. In the View app the joystick orbits the scene and zooms with its press held; the nav stick pans; the nav press clicked steps the camera mode and held resets the view; the joystick's press clicked resets the view, held goes home in the fixed mode.

### LEDs
| item | what it does | default |
|---|---|---|
| layout | V6 (16 x 30 + rails) or V5 (5 + 5 holes across the channel, 4 x 25 rail LEDs); a wired V5 chain forces V5 | V6 (V5 when the chain is found) |
| chain on *(mode)* | the V5 chain streaming (`N`); `test chain` runs a dot up it (`n`, which also prints the timing and the power line) | on |
| bright | `style.peak` - the cursor's peak level | 1.0 |
| strip | `stripBrightness` - a lever on everything sent to the chain | 1.0 |
| budget mA | the per-frame current budget: a frame that would draw more (12 mA per colour channel at full, an assumption) is dimmed whole, every LED by the same factor; never above the compile-time ceiling `PROBELED_STRIP_HARD_MAX_MA` = 2000 | 1000 |
| colours | classic / height / sure / amber / cyan / rainbow - the cursor's colour scheme; every scheme is white with the point on the board (below 1.5 mm) and takes its colour from 6 mm up | classic |
| full peak | the widest bell still keeps one LED at the peak (up to twice the narrowest width; wider, the peak fades with the width - see §4) | on |
| bloom | a halo three times as wide as the cursor at 0.3 x this | 0 |
| sparkle | random near-white flashes in the glow, more when lifted (density follows height) | 0 |
| pulse | breathing of the peak | 0 |
| fade | `decayS` - how long a lit LED takes to go dark (s); the attack is 20 ms | 0.12 |
| touch ring | a ring runs out from the point when it lands, and again at every new hole it slides to while down | off |
| V5 stream *(mode)* | the cursor as CSV on the console for a V5 to follow (`L`) | off |

### rows
`row mode` *(mode)* (`r`; on at boot) - which breadboard row the probe is over, on the LCD (the View and Rows apps) and console; `row` info; `calibrate 12 taps` (opens the Calibrate app; `c` on the console); `anchor at row` (the row stepped in place with left/right, then the press: `R<row>` with the probe in that row's hole next to the channel); `forget anchors` (`C`, asks first); `hold-still test` (`h`). A tap or a hold now lets a second go by while the hand settles, then measures two seconds. The grid the board boots with is `ROWCOUNT_GRID_AT_BOOT` (the 2026-09-17 calibration); the breadboard has moved since - it reads ~1.7 rows off at row 20 and puts a touching tip in the channel near the centre line - **anchoring is still to do**: `R20` in row 20's channel-side hole, `R50` likewise.

### play
| item | what it does | default |
|---|---|---|
| mode *(mode)* | off / paint / target (`y0`/`y1`/`y2`). The Draw app turns paint on when you go to it and off when you leave, the Target app likewise; the drawing stays in RAM and shows again on return | off |
| hue / sat | the brush colour: degrees round the wheel, and 0 (white centre) to 1 (rim). The draw screen's wheel sets them with the joystick | 0 / 1.0 |
| paint bright | the brush's level, 5 % steps; nav up/down in the Draw app. The brush's only: what is painted keeps the level it got | 0.5 |
| brush | 0-3 rows around the LED under the point, with a soft edge. **Newest wins** (`src/play/Paint.h`): a stroke paints over what was there, colour and level; within a stroke the centre beats an earlier edge and an edge never dims a centre; nav left/right in the Draw app | 0 |
| touch | the height (mm above the surface) below which the point paints, with 0.7 mm of hysteresis; also in the Draw app | 2.0 |
| clear | wipes the paint (`W`; the nav press held in the Draw app does too) | - |
| target | info: the target game's tally (`w` prints it) | - |

Draw app controls, in paint mode: **joystick** moves the wheel's marker (4 radii/s at full tilt; expo on the stick), **joystick click** toggles draw / ERASE (a click: on the release of a short press, never on a hold), **nav up/down** brightness, **nav left/right** brush, **nav press held** clear. The panel under the map shows every value; the map shows the drawing as the LEDs have it, and the brush ring (the LEDs just outside what a touch would paint) as hollow squares. In paint mode the LED cursor *is* that ring: no glow, bloom, sparkle or height colour, no fade.

### controls
| item | what it does | default |
|---|---|---|
| direction ms | how long one or two nav contacts must hold before they are a direction, and how long the push contact must be closed alone before it is the press | 20 |
| push guard ms | the same for a direction while the push contact is closed too - on this stick the push contact closes on every tilt, so this is the one that counts. A centre push wobbles into a direction contact first (up to ~100 ms was seen on 2026-09-18): raise it if a press ever moves the cursor, lower it for speed | 50 |
| joy menu at | how far the joystick goes (0-1 of its travel) before it counts as a menu direction | 0.25 |
| joy menu off | ...and how far back before that direction is over (hysteresis; kept under `at`, else 60 % of it) | 0.15 |
| joystick | `relative`: the stick is a four-way that steps the cursor. `absolute`: while the stick is going out its position IS the cursor - on Home the cell it points at (three bands each way, a third of the travel apart), on a Settings page the row among those on screen (its height); once it starts coming back the cursor stays where it was pointed (the stick springs home through the middle). The nav stick still steps either way; in an app the stick is the app's as before | relative |

How the stick and the buttons are read (`src/ui/StickDpad.h`, `src/ui/ButtonTracker.h`, `src/ui/Input.cpp`), after the practice in game UIs and embedded debouncing:

- The stick's **dead zone is radial and scaled** (Sutphin; Minimuino): inside 4 % of the travel it is nothing, the ring from there to 97 % is stretched to 0..1, so there is no step at the edge and no snap to the axes; the camera gets a squared magnitude on top (fine near the centre). The **four-way is axial** with the **dominant axis only**: a four-way wants one of four vectors and a diagonal push is one direction, never two.
- A four-way direction needs the stick past `joy menu at` for 30 ms (a click of the stick's own button tilts it a little, and the button going down cancels the crossing, so a click never moves the cursor first), and after a direction ends its axis is **disarmed until it has rested inside `joy menu off` for 80 ms**: a stick let go from full tilt springs back through the centre and rings, which at a low threshold fires the opposite direction (Godot's UI navigation hit this when its threshold went from 0.5 to 0.2: a false input on nearly every release).
- A **button's press is taken on the first sample** (no latency) and edges are then ignored for 20 ms; its **release needs 20 ms open**, since release bounce is the worse of the two and a hold must not end on it (Ganssle measured bounce up to 6 ms, most under 10; Hackaday's debounce series). The nav stick's contacts settle 10 ms before the decoder's guards, and the decoder's outputs are not debounced again. Held directions repeat after 400 ms, then every 80 (40 after twenty), the same shape as Unity's UI navigation (500/100).
- The Home grid's left/right wrap within the row; up/down stop at the edges.

The contacts settle 10 ms before any of this; nothing is debounced twice. `j` prints the numbers in force. A press selects on its way DOWN in Home, the menu and the dialogs (the release-based click felt like nothing happening).

### magnet, sensors
`strength` info, `learn strength` (`k`: hold the fit to this magnet's strength), `forget strength` (`K`), `re-zero (away!)` (`z`: the probe well clear of the board), `orientation check` (`o`), `latest fix` (`l`). `sensors` info, `array status` (`m`), `bus check` (`b`), `power-cycle` (`p`), `service table` (`X`), `saved settings` (`s`).

### root
`commands` (every console command as a menu action, but for `e v y B N r g u`, which have an item of their own or change the app), `reset settings` (asks first; the modes - row mode, the chain - are left as they are).

## 3. Console commands not on a page

`F` - **before a flash by hand**: stops the sampler and parks the other core (the V3F fetches the flash being written and the program fails otherwise). `pio run -t upload` sends it itself and waits for "parked". `Y0`/`Y1` - take the last good fix's magnet / the reference fix back out of a freshly taken zero. `d`/`f` - fix / field streams as CSV. `e` - next screen (scene, LEDs, log, draw; `e` also prints each screen's draw time). `v` - next camera. `g` - tracker on/off. `u` - cursor under/aim. `j`/`J` - the controls as read / the nav stick's contact trace. `B` - layout V5/V6. The README's table has them all.

## 4. Compile-time knobs changed in this session, and why

Values are the ones in the tree now. "Where" is the header.

**Sampling and the bus** (`src/magarray`)
- The sensor round-robin runs on the second core (`MagSampler`, `loop1()`), its hot path in ITCM (`__itcm_func`): 503 passes/s; each 100 Hz frame averages 3.3 conversions per sensor (the TMAG5273 at 32x averaging makes ~4 a frame: this is the ceiling). `MAG_FRAME_PERIOD_US` 10000. `SAMPLER_LOST_MS` 100 (a sensor is "lost" after 100 ms without a read, not after N failures).
- `MAG_ZERO_AT_BOOT` (`MagArrayConfig.h`): the zero used when none is saved.
- The wedged-block reset is done by the main core (the V3F's `SystemCoreClock` set SCL 4x too slow).

**Fit and smoothing** (`src/magfit/MagLocator.h`)
- `MAGLOC_FAST_MT` 2.0 / `MAGLOC_SLOWEST_ALPHA` 0.08: field smoothing off at 2 mT peak, heaviest (0.12 s) on faint signals.
- The "is it moving" speed for the smoothing: the larger of the distance over the last `MAGLOC_SPEED_WINDOW` = 10 frames and the filter's velocity, each less `MAGLOC_SPEED_JITTER_K` = 0.4 x what the fix's error bar makes of it. (The old fixed 5 mm/s dead band opened the smoothing on any resting probe; a hovering probe had 0.88 mm of jitter at rest on the bench, 0.13 now.)
- `MAGLOC_FLOOR_*`: the learned surface - window 1000 ms, 3 windows agreeing within 0.7 mm, at least 0.5 mm below the setting, fixes with misfit < 10 % and 5+ sensors only.
- `MAGLOC_REFERENCE_*`: the reference fix (row 35 hole 3, last night's mounting) for `Y1`.
- `MAGLOC_BOARD_Z_MM` 17.5, `MAGLOC_TIP_OFFSET_MM` 0, `MAGLOC_PRESENT_MT` 0.04.

**Tracker** (`src/magfit/MagTracker.h`): `MAGTRACK_ACCEL_SIGMA` 10000, `MAGTRACK_ROUGH_ACCEL_SIGMA` 2000, `MAGTRACK_GATE` 4, `MAGTRACK_SIGMA_FLOOR_MM` 0.3, the 1-Euro cutoffs/betas as in the smoothing page, `MAGTRACK_ROUGH_HOLD_S` 2.5, `MAGTRACK_ROUGH_SPREAD_MM_S` 8, `MAGTRACK_MAX_REACH_MM` 40.

**LEDs** (`src/probeled/ProbeLeds.h`, `ProbeLedService.h`)
- `PROBELED_MA_PER_CHANNEL` 12 (was 8: the cautious end for 1010 RGB LEDs), `PROBELED_STRIP_MAX_MA` 1000 (the budget's default), `PROBELED_STRIP_HARD_MAX_MA` 2000 (the ceiling). Scaled channels are truncated, not rounded (rounding up on 1200 channels was 28 mA past the line). Every path to the chain goes through `sendFrame`.
- `PROBELED_MAX_SIGMA_ROWS` 3 / `PROBELED_MAX_SIGMA_ACROSS_MM` 7: the glow is never wider; its floor fades out from twice the narrowest width to nothing at the widest (a probe pulled away shrinks into a dimming patch, not a disc over the board). `PROBELED_ROUGH_LEAST` 0.03: the far probe's faint "about here".
- `PROBELED_WHITE_BELOW_MM` 1.5 / `PROBELED_COLOUR_BY_MM` 6: white on the board, the scheme's colour from 6 mm up. `PROBELED_SPARKLE_TINT` 0.25, `PROBELED_SPARKLE_FLOOR` 0.1.
- The touch ring: `PROBELED_RING_S` 0.14 (its life), `PROBELED_RING_ROWS_PER_S` 36, `PROBELED_RING_WIDTH_ROWS` 1.5 (the LED on the ring at full, fading to nothing a row and a half out), `PROBELED_RING_DECAY_S` 0.02 (the LEDs it passed go dark at once, so it stays a ring), `PROBELED_RING_STEP_ROWS` 0.8 and `PROBELED_RING_REARM_S` 0.1 (a re-ring while sliding needs the point 0.8 rows from the last ring and the last ring 100 ms old). `PROBELED_TOUCH_MM` 2.0 / `PROBELED_LIFTED_MM` 5 (down / lifted for the ring).
- `LEDSTRIP_SPI_HZ` 3125000 (four SPI bits a WS2812 bit); the SCK pad is put back to an input and MOSI on the 10 MHz driver (the 3 MHz clock wedged the sensor bus). The symbol encoder is arithmetic, not a flash table (the table cost 2.6 ms a frame).
- `PROBELED_PERIOD_US` 20000 (50 Hz); the LED service runs ~0.5 ms.

**Paint** (`src/play/Play.h`): `PLAY_TOUCH_MM` 2.0, `PLAY_TOUCH_RELEASE_MM` 0.7, `PLAY_WITHIN_ROWS` 1.0 (a point up to a whole pitch from a hole still paints it - near the channel the fit puts a touching tip up to 2 mm into the channel), `PLAY_BRUSH_MAX` 3, `PLAY_BRIGHT_STEP` 0.05, `PLAY_PICK_PER_S` 4, `PLAY_TRACE_MID_Y` 92 (the map's channel line on the draw screen).

**Rows** (`src/rowcount/RowCounter.h`): `ROWCOUNT_TAP_SETTLE_FIXES` 100 + `ROWCOUNT_TAP_MEASURE_FIXES` 200 (a tap: 1 s settling, 2 s measured), `ROWCOUNT_HOLD_SETTLE_FIXES` 100 + `ROWCOUNT_HOLD_FIXES` 200 (an `R` anchor or `h` likewise), `ROWCOUNT_HOLD_TIMEOUT_MS` 8000, `ROWCOUNT_ROW_MODE_AT_BOOT` true, `ROWCOUNT_GRID_AT_BOOT` (the 09-17 grid).

**Display** (`src/display/MagView.h`): `MAGVIEW_STARVE_US` 60000 - a screen not drawn for 60 ms is drawn whatever the LED timing says (the LED-first rule used the last draw's time as the expectation; one draw stalled by a settings write froze the screen, twice). Draw times (measured on the night, `e` prints the current ones): scene ~3.6 ms, log 1.5 ms; text and lines go through `FastDraw`, not GFX.

**Settings** (`src/settings/Settings.h`): `SETTINGS_CHECK_MS` 100 with `SETTINGS_STEPS_PER_RUN` 16 (the serialisation in steps, ~1 ms a run instead of 5.5 ms every 2 s), `SETTINGS_SETTLE_MS` 2000, `SETTINGS_TUNING_VERSION` 3.

**Input** (`src/board/BoardPins.h`, `src/ui/Input.h`): `JOY_X_REVERSED` 1 (the stick as wired), `INPUT_NAV_DIRECTION_WITH_PUSH_MS` 130 (a nav direction must hold 130 ms while the push contact is closed, so a push's wobble is not a direction).

**Flashing** (`scripts/park_before_upload.py`, `platformio.ini`): `pio run -t upload` sends `F` and waits for "parked" before wlink runs. By hand: `F` on the port, then `wlink flash`. If wlink answers `0x55` to everything the chip has a half-written image: hold RESET through the flash (the morning report, §1).

## 5. Things to know that are not knobs

- The V3F (core 0, 100 MHz) has no instruction cache and its `micros()` has no sub-millisecond part; code it runs in a loop goes in ITCM, and `millis()` is its clock. A `static const` table read per LED on the main core is a flash data read the I-cache does not help with.
- The joystick's push read high at rest on the bench of 2026-09-19 (`j` shows it): an ordinary switch to ground, `JOY_PRESS_ACTIVE_LOW 1` in `BoardPins.h`. A click is the release of a press shorter than 500 ms (`src/ui/ButtonTracker.h`); a hold fires at 500 ms and is never also a click.
- `m` pauses the sampler for a 2 ms diagnostic read; `X` is the service table (runs, avg/max µs, overruns) - the first place to look when anything feels slow.
- The pencil bench: `tools/hostsim/pencil.cpp` (`far` for a hovering hand); the screens simulator `tools/hostsim/screens.cpp` renders every screen to PNG and checks the settings and the paint; `tools/fixstats.py` reads a `d` stream's rest statistics.
- Unit tests: `pio test -e native`. The LED tests encode today's glow rule (the widest bell fades), not last night's (the widest bell keeps a full LED).
