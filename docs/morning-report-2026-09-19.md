# Morning report, 2026-09-19

The night's brief: probe resting in row 35, hole 3, at a steep angle; make the tracking, smoothing and sensor fusion as good as they can be, free the main core, add modes to feel the tracking. Written at 01:10 while the board is unreachable (first section), then updated as the rest was finished.

## 0. 07:31: the board is back

The retry loop's 4252nd `wlink flash` went through at 07:31:09 (either a lucky window or a hand on the board - the probe was moving a minute later). Checked at 07:33 on the recovered board: boot without "row mode off" / "LED strip off"; the sampler at **531 passes a second**, 0 fails, 0 block resets; the Settings service at 0.9 ms a run (was 5.5); the `track:` line as intended ("velocity 19 mm/s (moving 11, smoothing alpha 0.46)" with the probe in hand); the LED chain streaming at 50 Hz, 0 refused. All four untested items have now been seen working (the `F`-then-flash path at 07:42).

**07:36-07:42, the white LEDs.** The settings had been wiped by the recovery, so the array zeroed itself at boot with the probe lying near the board: that zero was 0.15 mT off on four sensors, every fit after it had 20-50 % misfit (2 % all night), the height read 5 mm low with the tip in the air, and the LEDs painted white. `z` with the probe held clear fixed it (the new zero matches last night's within 0.02 mT). So that this cannot happen again the board no longer zeroes blind: when no zero is saved it boots with the compiled-in `MAG_ZERO_AT_BOOT` (this morning's clean one) - flashed at 07:42 through `pio run -t upload`, which also proved the park-before-flash path (F sent, flash done first time with the board running). The menu values are back at their defaults and the row anchors at the boot grid; both are saved again as they are changed. Also noticed: the LED service now takes ~4 ms a run (it was 0.6) - probably the default effects on a fully lit chain; not urgent, but worth a look on the LEDs page.

## 1. What happened at 00:51 (kept for the record)

At 00:51 a flash (`wlink flash`) failed part way through - `Error while fastprogram: [41, 01, 01, 05]`, which happened every other flash during the night and always recovered by waiting for the WCH-LinkE to re-enumerate and flashing again. This time it did not: since then every probe operation (`status`, `erase`, `reset`, `halt`, at any speed) returns `WCH-Link underlying protocol error: 0x55`, and the serial port prints nothing at all. The chip is most likely running a half-written image and not answering the debug module (the Arduino core's `docs/hazards.md` describes exactly this state; it is what a fault that reproduces on every boot looks like from the probe's side).

Tried from here, none of it worked: four libusb resets of the WCH-Link; a DAP-mode switch and back (a full re-enumeration of the probe); the hub's per-port power switch (`uhubctl`, installed with brew for this - the hub says the port is off but the device never actually dropped); the probe's own 3V3/5V outputs off and on; `erase --method pin-rst` and `--method power-off` (the nano has neither wired to the probe); forty attach attempts in a row.

What the core's hazards notes say works, in order:

1. **Hold the board's RESET (NRST) button down, run the upload, then let go:**
   ```sh
   cd ~/Documents/GitHub/Testolomew
   pio run -t upload
   ```
   (`pio run -t upload` is now the way to flash - it first sends the console command `F`, which parks the other core; on a dark board that is a no-op. By hand the same is `pio run` then `~/.platformio/packages/tool-wlink/wlink flash .pio/build/testolomew/firmware.elf`. A flash only erases the pages it writes, so the settings - the zero, the row anchors, every menu value - in the flash tail survive it. If wlink still answers `0x55`, `~/.platformio/packages/tool-wlink/wlink --chip CH32H417 erase` with reset held, then the upload; the settings are then gone, and with the probe lying on the board at boot the zero eats the magnet: `Y1` puts the known reference fix back into the baseline, or lift the probe off and `z`.)
2. If the probe still says `0x55`: unplug the board's USB completely, wait ten seconds, plug it back in, and do step 1 again. Power-cycling the board alone is said not to clear a wedged probe; the replug does.
3. The other door: hold BOOT while plugging in the chip's own USB-C (the OTG one), and the ROM bootloader enumerates; `~/.platformio/packages/tool-wchisp/wchisp flash .pio/build/testolomew/firmware.bin`.

The tree builds (`pio run`), the unit tests and the host simulator pass, so what goes on is what is described below. **Not yet run on the board** (everything after 00:51 - the firmware is frozen from 02:00 so the list stays short): the stepped settings serialisation; the smoothing's new speed rule (`MAGLOC_SPEED_JITTER_K`); the `F` park-before-flash command and the upload hook; the `track:` console line now printing "velocity ... (moving ..., smoothing alpha ...)" in place of "speed". If the first boot after the recovery looks wrong, those four are the suspects, in that order. (A retry loop is trying `wlink flash` once a second and stops when one goes through - `retry_flash.sh` in the job's tmp folder, `~/.claude/jobs/bcb6e98c/tmp`, log `retry_flash.log` there. The chip does attach now and then - a `status` at 01:21:55 and a `halt` at 01:23:54 went through, and then 0 of the next 120 tries - so it may well be running when you read this; if it is, that is what happened, and the loop has exited. If it is not, `pkill -f retry_flash.sh` before step 1 so the two do not fight over the probe.) The last firmware that actually ran on the board (00:36 to 00:51: 503 sampler passes a second, LED chain streaming, zero bus resets, row 35 read as 35.16 +/- 0.12) is everything below except the stepped settings serialisation (section 3, last item), which is what the failed flash was carrying.

**Why the flashes were failing, found afterwards (01:55).** The Arduino core's `ch32h4_park.c` says it outright: the V3F has no instruction cache, fetches every instruction from the flash being programmed, and a page program with it running does not complete. The sensor sampler has run on the V3F since 23:30, so every flash since then raced that core, and about every other one lost. The firmware's own flash writes (the settings) park the V3F first, by interrupt; a flash from outside did not. Now there is a console command `F` that stops the sampler and parks the V3F in ITCM for good (a reset brings it back), `pio run -t upload` sends it before wlink runs (`scripts/park_before_upload.py`), and flashing by hand you send `F` first. Step 1 above holds either way for the state the board is in now; once it is running this build, flashes should stop failing.

## 2. A boot bug worth knowing about

Every boot since the settings module went in (yesterday evening) had switched **row mode and the LED chain off** one second after start-up: the settings load carries the menu's toggle items into the modules, and the two it deliberately does not save (they are modes) sat at their compile-time `false` and were carried in too. `row mode off` and `LED strip off (cleared)` were in every boot log. So the "default to row mode" and "LEDs on" asks had been silently undone at every reset, and the LED chain was off for the whole first half of the night's recordings. Fixed (`Ui::settingsLoaded` reads those from the modules first); the boot log no longer shows either line.

## 3. What changed, with numbers

Before/after is the probe as you left it, at rest. The "before" recording is the firmware as it was at 23:04 (`tools/recordings/2026-09-18-night-baseline-fixes.txt`, ten minutes); the "after" is 00:19 (`2026-09-19-night-final-fixes.txt`, 80 s, the last one taken before the board went dark). Most likely both with the LED chain off (section 2: the firmware of the evening switched it off at every boot - the capture of the boot right after the baseline, 23:17, shows it - and the before/after `m` outputs show no bus resets either side), the bus healthy (one block init at boot, none since). `tools/fixstats.py <file>` prints these.

| at rest, probe in row 35 hole 3 | before | after |
|---|---|---|
| raw fix jitter, robust (MAD) x / y / z | 0.10 / 0.18 / 0.13 mm | 0.10 / 0.18 / 0.13 mm |
| raw fixes more than 1 mm off, x / y / z | 91 / 78 / 445 of 7960 | 2 / 1 / 0 of 1590 |
| raw fix sd x / y / z | 1.72 / 0.90 / 0.49 mm | 0.13 / 0.17 / 0.14 mm |
| tracked position sd | 0.45 / 0.79 / 0.46 mm | 0.14 / 0.23 / 0.17 mm |
| cursor sd x / y | 0.41 / 0.85 mm | 0.10 / 0.17 mm |
| magnet axis wander (mean angle from the mean axis) | 2.1 deg | 1.1 deg |
| tilt | 41.3 deg | 41.3 deg |
| misfit | 4.5 % | 2.1 % |

So the noise floor of a single fix (the MAD) is the sensors' and did not move; what went away are the excursions - 5.6 % of fixes were more than a millimetre off in height before, none after - and half the angle wander. Those excursions were the main core's bus holds and dropouts, and they are what a cursor shows as a twitch.

What did it, in the order it went in (`docs/wireless-probe-sensing.md`, the "night of 2026-09-18/19" entry, has the detail on each):

- **Sensor sampling on the other core.** The I2C round-robin over the eight sensors runs on the V3F (`src/magarray/MagSampler`, `loop1()`), reading the block's registers directly into shared RAM; the V5F only decodes what is new. MagArray on the main core: 1.6 ms a run to 65 us. Then the V3F turned out to be fetch-bound (no instruction cache, flash at a 25 MHz equivalent): 158 passes a second. The read's hot path and `loop1` are in ITCM now: **503 passes a second** with the LED chain streaming, 0 block resets, each 100 Hz frame the average of 3.3 reads of every sensor (was 1.6, and 2.9 on the main core before the night).
- **The console by DMA** (`src/console/SerialDma`): `Serial.write()` waited for every byte; a row line held the loop 13 ms. Fit rate 74 to 99 a second; frames on a true 100 Hz grid.
- **Tracker and smoothing tuned on a bench** (`tools/hostsim/pencil.cpp`: a hand that writes at 50-250 mm/s and turns the pencil, through the real locator and tracker, reporting lag and rest jitter). The shaft direction through its own 1-Euro filter (the angle used to lag the position); rough (far) fixes tracked instead of followed. On the build that waits for the board, writing on the board: the track 10 ms behind the hand, the scene's magnet and the cursor 20 ms, 0.07 mm of jitter at rest (the cursor 0.02), the shaft 0.6 deg at rest and 1.8 deg while turning at 1 rad/s; hovering 12-22 mm up: 30-40 ms behind, 0.13 mm at rest.
- **The zero survives a reboot** and `Y` takes a known magnet back out of a fresh zero (`Y1` = the reference fix from this night's mounting). No more reflash-with-the-probe-on-the-board eating the magnet.
- **Sampler robustness**: lost = no read for 100 ms rather than a fail count; the block reset done by the main core (the V3F's clock is not what it thinks).
- **Play modes** (`src/play`, menu page "play", console `y1`/`y2`, `w` score, `W` clear): *paint* - the point paints the LED it touches, six colours and a brush; *target* - a green LED to touch, tallying the time to reach it and the miss in mm; and the *draw screen* on the LCD (`e` cycles to it) - the point's path over a map of the board. These are for feeling the tracking rather than reading it.
- **The boot bug** above.
- **Park before flash** (`F`, and `pio run -t upload` sends it): see section 1. The cause of the night's flash failures and of the board being dark.
- **Settings serialisation in steps** (`src/settings`): the check that watches for changed settings held the loop 5.5 ms every 2 s formatting floats - the LED chain's longest gap was 28 ms against a 20 ms mean. It now does 16 records per 100 ms run (a full text about once a second), so no run holds the loop more than ~1 ms. Built and host-simulated; **not yet run on the board** (this is the build the failed flash was carrying). If anything odd shows in `s` or a setting does not stick after 2 s, this is the suspect.

Earlier in the evening, before the night: text and lines drawn straight into the framebuffer (`src/display/FastDraw`: the log screen 45-60 ms to 1.5 ms, the scene 12-18 to 3.6 ms), the LED chain's SPI clock pad quieted (it was wedging the sensor bus: ~50 resets a minute to 0), the LED current budget, the white-at-touch and sparkle rules, the nav-switch press timing.

## 4. What did not happen

- The planned final five-minute recording at 05:45 and the long log after 00:51: the board was dark. The long log (`tools/recordings/2026-09-19-night-long-run.txt`, a fix every 15 s) covers 00:20-00:50: x 1.23 +/- 0.11, y 9.17 +/- 0.17, z 10.75 +/- 0.16 mm, tilt 41.6 +/- 1.4 deg over the half hour, no drift.
- Nothing is committed (not asked). `git status` shows the night's work; the new files are `src/play/`, `src/probeled/`, `src/settings/`, `src/magarray/MagSampler.*`, `src/magfit/MagTracker.*`, `src/console/SerialDma.*`, `src/display/FastDraw.*`, `scripts/park_before_upload.py`, `tools/hostsim/pencil.cpp`, `tools/fixstats.py`, the recordings, and this report.

## 5. Open items, in the order I would take them

1. *(done at 01:40, on the bench, not yet on the board)* **The tracker's "speed" at rest read ~15 mm/s** (the `track:` line) on a probe that was not moving: the filter's velocity is the fix jitter divided by 10 ms, and the field smoothing's "is it moving" rule keyed off it with a 5 mm/s dead band, so it opened the smoothing on any resting probe - on a far one until the smoothing was defeated by its own noise (bench: 0.88 mm of jitter at rest, hovering). The speed is now the larger of the distance come over the last ten frames and the one-frame velocity, each less 0.4 x what the fix's own error bar makes of it. Bench: far at rest 0.88 to 0.13 mm of jitter, far moving +20 ms of lag, near the board unchanged (track lag 10 ms). Unit tests and the ten-minute soak pass (track error in range 1.07 to 0.92 mm rms). It is in the build that waits for the board; the glow should sit still with the probe held in the air, and the `track:` line's "moving" figure should read 0 at rest.
2. The four untested items of section 1 on hardware: the first `s` after a menu change (2 s later it should say one write), a flash through `pio run -t upload` with the board running (it should print "park_before_upload: sent F" and not fail), the sampler back at ~500 passes a second after that flash (`m`).
3. `tools/hostsim/pencil.cpp` reports lag against the truth for the cursor, the scene and the shaft; what it cannot measure is how the lag *feels* against the LEDs, which only a hand can. The paint mode is the test: a stroke should leave a line under the point, not behind it.
