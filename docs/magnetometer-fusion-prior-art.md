# Sensor fusion for the Jumperless magnetic probe: prior art and design

Everything found on the night of 2026-09-17 about getting a better position out of the TMAG5273 array than one frame's dipole fit gives: the chip read register by register, what other people have done with magnets and sensor arrays, the closed forms that work at range, the motion filters that suit a hand, how the cursor should point, how the breadboard LEDs and the LCD should show it, and a firmware design that puts it together.

Compiled from seven research notes written that evening (TMAG5273 datasheet, TMAG5273 field notes, dipole tracking and fusion, far-field closed forms, motion filtering and pointing, LED uncertainty display, small-display UI and camera), then revised the same night after a review against the sources, with eight gap notes (`sections/gap-1.md` … `gap-8.md`: the tracker's at-rest numbers, the far-field numbers re-run with the real magnet and raw noise, the V6 dual-chip data path, the board's own magnetic sources, the E2E record read verbatim, the frame budget and dt, the kinematic glitch rule, and the surface-plane frame). Each note marked its claims as read in the source, seen only in an abstract, inferred, or recalled from memory; those marks are carried here as *(verified)*, *(abstract)*, *(inferred)* and *(recalled)*, and a number without a mark comes from a verified source or from our own measurements in `wireless-probe-sensing.md`. Working files (PDFs, text dumps, the twenty-four saved E2E threads, the simulation scripts and host benchmarks named below) are in `~/.claude/jobs/bcb6e98c/tmp/research/`.

**Status in one paragraph.** The fit is noise-limited close in and floor-limited everywhere: about 0.05 mT of repeatable, unexplained field error on the bench array is five times the per-frame noise, and every estimator in the literature bottoms out on that kind of error, not on the filter. So the order of work is to measure the loop first (the frame is 12–13 ms today, not 10, and the 49 ms "cold start" is a fallback path that tracking frames also fall into, §2.4 and §4.4), then calibration (per-axis gains, a live baseline, honest sample rejection from the chip's own status byte, and on V6 a known-input correction for the LED supply currents, §2.11), then the constant-velocity Kalman track that already exists in the working tree as `MagTracker`, retuned with a chi-square gate that down-weights before it drops (Huber), a process noise that does not lock the track out of an ordinary reach, no acceleration or reversal rule, and an unconditional human-speed bound (§5.2–5.4; two candidate settings, the innovation log decides), then the surface-plane cursor in the fit frame (§6), then the LED and LCD layers, which on V6 live on the RP2350 at the far end of a 34-byte message per fit (§9.5). With the bench magnet (moment ≈ 4,500 mT·mm³, not the 9,800 first assumed) and raw per-frame noise, every far-field range in §4 is 15–25 % shorter than the first draft said: fixes to ~60 mm, presence to ~80 mm, and the proposed V6 4 × 6 mm N52 rod buys the 100 mm back. A field-space EKF is the fallback if the track loses to the per-frame fit on the weak-signal recordings, and the Nara closed form, though verified, has already lost to a lattice search on this array's geometry.

---

## 1. What was asked, and where the tracker stands

Kevin's list for the night: prior art on sensor fusion for the magnetometers; the TMAG5273 datasheet read thoroughly; a rough estimate when the probe is far away; a way to throw out motion a hand cannot produce; inferring position from past movement; a setting for the cursor to follow where the tip *points* rather than what is under it; a virtual plane at the breadboard surface (7.1 mm above the base PCB on V6) where the pointing ray stops; LED driving on the 16 × 30 grid (a V6 layout with six holes to the centre and a V5 layout with the 5.08 mm channel) that follows the probe and shows uncertainty subtly; and an LCD with camera perspectives (pan, POV from the tip) and an on-screen menu driven by a nav switch, a joystick and two buttons, able to do everything the serial console does, including showing its output.

**The tracker tonight.** A 4 × 2 TMAG5273 array (gaps of 16 / 22 / 16 mm along each row, rows 44.3 mm apart after self-calibration) on a nanoCH32H417 at 100 Hz frames. A single-dipole Levenberg–Marquardt fit with the moment solved linearly (variable projection) gives position, moment direction (the shaft, since the magnet is magnetised along it and the tip is a known offset down it), a 1-σ error bar per axis from s²(JᵀJ)⁻¹ and a misfit ratio. Over the array at 16 mm height the bar is 0.04–0.10 rows (0.1–0.25 mm) and the row is right; over the 10 mm overhangs it is 0.2–0.35 rows; at 30–40 mm height it passes a whole row. Per-frame noise is 0.011 mT on X/Y and 0.006 on Z after the in-frame averaging, the array carries the ~0.05 mT repeatable error described in `wireless-probe-sensing.md` §6.1 ("no fit" paragraph), fields are 0.1–0.3 mT far from the array and tens of mT close, and fixes drop out now and then from bus glitches and weak signal at the edges. The bench magnet's fitted moment is ≈ 4,500 mT·mm³ (`wireless-probe-sensing.md` §6.1), which is what every "our geometry" number in §4 now uses. A converged tracking fit is 2–3 LM iterations, ~0.3–0.5 ms on the V5F (inferred from the two-magnet measurement; `fitUs` in the `d` stream is the number to log). The 49 ms measured for a cold start is the seven-seed fallback crawling to 30 iterations per seed, which the code also enters from a *tracking* frame whose misfit exceeds 40 %, so on this array at 40–50 mm every frame can cost it; the lattice-plus-one-refine cold start is ~2–5 ms by the same scaling (§4.4). And the frame is not 10 ms: measured, the loop publishes every 12–13 ms (§2.4).

**What is already in the working tree (uncommitted as of tonight).** `MagLocator` already carries `pointer` (the shaft carried on from the tip to `MAGLOC_BOARD_Z_MM`, 17.5 mm above the sensors on the bench), a *rough* fix class (`MAGLOC_ROUGH_MAX_MISFIT 0.7`, `MAGLOC_ROUGH_MAX_ERROR_MM 60`, `MAGLOC_ROUGH_MIN_SIGMA_MM 3`) for a far probe, and a baseline-pollution check that fits each new baseline as a dipole and refuses it if a magnet of ≥ 300 mT·mm³ explains it. `MagFit` has a lattice search (`magFitCoarse`: 25 mm steps, 45 mm margin, heights 4–110 mm, halving to 3 mm, ~250 trial positions) that starts every cold start. `MagTracker` (`src/magfit/MagTracker.h`, host-tested) is a three-axis constant-velocity Kalman track with a σ-floor, a per-σ gate, a reinit rule, coasting with velocity decay, separately smoothed shaft, a surface-plane cursor in UNDER or POINTED mode with a reach cap, and a 1€ filter on the cursor. A closed-form gradient estimator (`MagFar`, Nara's formula) was built, measured on this array, and moved to `attic/far-field-gradient/`. Section 9 treats those as the base and says where the research argues for different numbers.

## 2. The TMAG5273 in detail

Source: TI SLYS045C, June 2021 revised April 2026, read in full (`tmag5273.pdf`, `.txt`). E2E forum threads were read in a real browser (TI's bot wall blocks scripted fetches) and saved verbatim under `research/e2e_<n>.md` with title, canonical URL and responder names; twenty-four are on disk and every quotation attributed to a TI responder below was checked against them word for word, the rest are marked *recalled* (the list is in §10). Threads are cited by number under `e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/<n>/`. CRC arithmetic was re-run in Python against every byte string the posters printed.

### 2.1 The part

Three Hall elements (X and Y vertical, Z horizontal) multiplexed through one signal chain and one 12-bit ADC, on-chip averaging to 32× giving a 16-bit two's-complement result per axis, a die temperature sensor on the same ADC, I²C to 1 MHz with optional CRC, an INT pin that can be an output (result ready, threshold) or an input (conversion trigger), a two-axis CORDIC angle engine, four power modes. Grades x1 (A1–D1) ±40/±80 mT and x2 (A2–D2) ±133/±266 mT; four factory addresses per grade. SOT-23-6, 1.7–3.6 V (DS §1, §3, §6.1).

**Where the element is** (DS Fig 6-2, p. 13, verified from the rendered page): 0.68 mm from one long body edge (0.12 mm off the body's centre line across its 1.6 mm width), 1.85 mm from the pin-1 end (about 0.4 mm off centre along the 2.9 mm length, toward pins 3/4), 0.73 mm above the seating plane. TI's advice, verbatim: "You should use the average dimensions when determining the location/offset of the hall sensor element in the package" (Rosenberger, E2E 1309963), i.e. the 0.12 mm across, not the 0.195 mm the maximum body width would give. `magcal` absorbs this on the bench; hand-entered V6 positions should use the element, not the pad centroid, and an underside part puts its element at −(PCB thickness + 0.73 mm) below the top surface. **On V6 the parts are proposed for the underside of the 1.6 mm MainBoard** (§6, §9.3), which puts the elements at z_el = −2.33 mm below the MainBoard's top surface; the fit frame itself has z = 0 at the elements whichever side they are on (`MagArrayConfig.h`: every sensor in the table has z = 0, and the `underside` flag only mirrors axes). Positive codes for a north pole approaching along +axis (DS §6.3.1), which is the opposite of the firmware's board-frame convention; the driver converts.

### 2.2 Register map

Addresses hex, reset values from DS §8.1.

| Addr | Register | Reset | Fields |
|---|---|---|---|
| 00 | DEVICE_CONFIG_1 | 00 | [7] CRC_EN. [6:5] MAG_TEMPCO: 00 none, 01 0.12 %/°C NdFeB, 11 0.2 %/°C ceramic. [4:2] CONV_AVG 000 1× … 101 32×. [1:0] I2C_RD: 00 standard 3-byte read, 01 1-byte read of 16-bit data + CONV_STATUS, 10 1-byte read of 8-bit MSBs + status. |
| 01 | DEVICE_CONFIG_2 | 00 | [7:5] THR_HYST (000 signed threshold, 001 symmetric ±band). [4] LP_LN: 0 low-power 2.3 mA, 1 low-noise 3.0 mA. [3] I2C_GLITCH_FILTER: 0 = filter ON (note the polarity). [2] TRIGGER_MODE: 0 I²C trigger bit, 1 INT pin. [1:0] OPERATING_MODE: 00 standby/trigger, 01 sleep, 10 continuous, 11 wake-up and sleep. |
| 02 | SENSOR_CONFIG_1 | 00 | [7:4] MAG_CH_EN: 7 = XYZ; 8–B are the pseudo-simultaneous two-axis modes XYX, YXY, YZY, XZX. [3:0] SLEEPTIME 1 ms … 20 s. |
| 03 | SENSOR_CONFIG_2 | 00 | [6] THRX_COUNT 1 or 4 crossings. [5] MAG_THR_DIR. [4] MAG_GAIN_CH. [3:2] ANGLE_EN. [1] X_Y_RANGE: 0 ±40 (x1) / ±133 (x2), 1 ±80 / ±266 mT. [0] Z_RANGE likewise. |
| 04–06 | X/Y/Z_THR_CONFIG | 00 | signed 8-bit threshold, LSB = 40·(1+range)/128 mT on x1. 0 = off. |
| 07 | T_CONFIG | 00 | [7:1] temperature threshold, 8 °C/LSB. [0] T_CH_EN. |
| 08 | INT_CONFIG_1 | 00 | [7] RSLT_INT. [6] THRSLD_INT. [5] INT_STATE 0 latched / 1 10 µs pulse. [4:2] INT_MODE 001 INT pin, 011 SCL. [0] MASK_INTB (1 = INT disabled). |
| 09–0B | MAG_GAIN_CONFIG, MAG_OFFSET_CONFIG_1/2 | 00 | angle-engine gain (value/256) and two signed 8-bit offsets, 19.5 µT/LSB at ±40 mT. Two channels only. |
| 0C | I2C_ADDRESS | 6A | [7:1] address, reloaded from OTP at every power-up; [0] I2C_ADDRESS_UPDATE_EN. |
| 0D | DEVICE_ID | | [1:0] VER: 1 = ±40/80 part, 2 = ±133/266. |
| 0E/0F | MANUFACTURER_ID | 49 / 54 | "I" "T". |
| 10–17 | T, X, Y, Z MSB/LSB result | 00 | 16-bit. |
| 18 | CONV_STATUS | 10 | [7:5] SET_COUNT rolling conversion-set count. [4] POR (W1C). [1] DIAG_STATUS (VCC UV, memory CRC, INT or clock error). [0] RESULT_STATUS. |
| 19–1B | ANGLE_RESULT, MAGNITUDE_RESULT | | 13-bit angle in 1/16°, 8-bit magnitude. |
| 1C | DEVICE_STATUS | 10 | [4] INTB level; [3] OSC_ER, [2] INT_ER, [1] OTP_CRC_ER, [0] VCC_UV_ER, all W1C. |

Scaling (DS Eq 10): B_mT = code16 / 2¹⁶ × 2·BR, i.e. 1.2207 µT/LSB at ±40 mT (820 LSB/mT), 2.441 µT/LSB at ±80 (410 LSB/mT). Temperature (DS Eq 12, §5.6): T = 25 + (code − 17508)/58 °C, noise 0.4 °C rms at 1×, 0.2 °C at 32×. The lower four result bits are only meaningful with averaging enabled; without it they read zero (E2E 1613640, TI-confirmed).

### 2.3 I²C: addresses, frames, CRC

**Addresses** (DS Table 6-2): A = 35h, B = 22h, C = 78h, D = 44h, the same for both grades. The address register is RAM loaded from OTP at every power-up; "at each power cycle these bits must be written again" (DS §8.1.13, §7.2.2), and "there is not a way to permanently change the default I2C address" (E2E 1661100). The address survives sleep and wake-and-sleep modes (DS Table 6-5). If VCC ramps slower than 3 V/ms, run one wake-and-sleep cycle after power-up "to avoid I2C address glitch during sleep mode"; not needed in standby or continuous mode (DS §5.3 note 1). TI confirmed that SDA, SCL and INT may sit at 3.3 V while a part's VCC is off (E2E 1394437), which is what the GPIO-powered addressing walk relies on.

**Frames.** Write: `S addr+W, [TRIG | reg7], data…`; bit 7 of the register byte triggers a conversion (DS §6.5.1.3.1). A general-call write (address 00h) reaches every device at once, trigger bit included (DS §6.5.1.3.2), "useful to configure multiple I2C devices in a I2C bus simultaneously". Standard read: `S addr+W, [TRIG|reg]; Sr addr+R, data…` (DS §6.5.1.3.3). The 1-byte 16-bit read (`I2C_RD = 01`, what the driver uses): `S addr+R` then [T] X Y Z as MSB/LSB pairs and CONV_STATUS, enabled channels only (DS §6.5.1.3.4). There is no register byte in that frame, so no trigger bit, and register reads stop working in that mode; `tmag5273Begin` switches back before it configures.

**Timing** (DS §5.10): 400 kHz from 1.7 V; 1 MHz needs 2.3–3.6 V, ≤ 50 pF, rise ≤ 120 ns, fall ≤ 55 ns, SCL high ≥ 350 ns, low ≥ 500 ns.

**Result atomicity** (DS Table 6-1, and TI's Jesse Baker in E2E 1259072): while the bus is talking to the device the result registers and SET_COUNT do not update; a read during a conversion returns the previous set. A torn MSB/LSB pair cannot come from the sensor, so a corrupt reading is a bus-level event. The result registers clip in an over-range field rather than wrap.

**CRC: what the datasheet says and what the silicon does.** Polynomial x⁸ + x² + x + 1, initial FFh (DS §6.5.1.3.6). An ordinary CRC-8 (poly 07h, init FFh, MSB first, no reflection, no final XOR) reproduces all seven datasheet examples (00→F3, FF→00, 80→7A, 4C→10, E0→5D, 00000000→D1, FFFFFFFF→0F) *(verified by computation)*:

```c
static uint8_t crc8_tmag(const uint8_t* d, int n) {   // poly 0x07, init 0xFF
    uint8_t crc = 0xFF;
    for (int i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}
```

In 3-byte mode the datasheet (Rev C §6.5.1.3.3) says the device "sends the fifth CRC byte based off the CRC calculation of immediate past four register bytes"; in 1-byte mode it covers "the command byte and the data sent in the current packet". **TI's supported CRC configuration is 3-byte mode with at most four data bytes per block**: Scott Bryson, after taking it to the design team (E2E 1200004, *verified*): "there is an errata in the data sheet. To use CRC with this device, it needs to be in the full 3 byte read mode and must capture no more than 4 consecutive data bytes … If this read mode is required, TMAG5173-Q1 operates with CRC in the 1 byte read mode for both 8 and 16 bit read modes." No errata document was ever published; Rev C's revision history says it *deleted* the earlier "not supported above 4 bytes" note ("Removed the exception note with CRC support for long data stream") and added "the first CRC can be incorrect after switching communication between devices" (§6.5.1.3.6). The datasheet still prints the 1-byte CRC frames.

What the silicon does, from the posted frames (*verified*, arithmetic re-run against every byte string the posters printed): **every wrong 1-byte-mode CRC is the datasheet CRC with exactly one extra byte in front.** In the two-device capture (Steve Melnikoff, E2E 1558871 / 1592349, A2 + B2 on one bus, trigger-write then read per device) that byte is the *other* sensor's 8-bit write address from its trigger transaction: 0x44 (= 0x22 ≪ 1) is the only single prefix out of 256 that reproduces device A's CRC, swapping the bus order swaps the roles, and prefixing the whole foreign transaction does not match, so only the address byte is folded in. Reads preceded only by the other device's *read* were clean in those captures. But a single-device continuous-mode case (Frank Pilhofer, E2E 1200004, reads only, 100 kHz) was correct on the first read after power-on and wrong on every later one, explained by an unattributable constant prefix 0xDB and by nothing else tried, so whether a read-only round-robin is clean is **open**. TI could not reproduce either case and offered failure analysis. Melnikoff's fix, "does the tx and rx for device A first, and then repeats this for device B. This appears to solve the problem", made 1-byte CRC reliable (user-confirmed), and one device in continuous mode, 32×, CRC on, read every 10 ms, ran "more than 15h … 0 communication error, nor CRC mismatched" (Jaermann, E2E 1171980).

In 3-byte mode the captures are cleaner: a 6-byte read of X Y Z from 12h returned a CRC over exactly those six data bytes on all five captured frames, never over the address or command byte and never over four (Cacula, E2E 1357408; Rosenberger: "If this seems to be working for you, then it should be okay. However, this is not the recommended use case and this operation is out of spec"), and Simmons' EVM figure (CRC over `44 F0 00 1E` = 0x24, E2E 1232741) checks. Where the CRC byte lands matters, because the device cannot know how many bytes the master will read, and the four 3-byte-mode reports fit one hypothesis, **a CRC after 2 × (enabled channels, T included) data bytes**: Cacula, XYZ, CRC after six; TI's test engineer in 1232741, `SENSOR_CONFIG_1 = 30h` (X, Y) with T off, after four; a user in the same thread with T *on*, whose 5-byte read returned Y_MSB where he expected the CRC on two boards and two hosts, which TI never explained and T_CH_EN would; and Roland Wiget's "When I read out 10 bytes, I get a CRC byte after every 4 bytes" (E2E 1121557), a customer's prose with no capture that conflicts with 1357408. Rosenberger's "the TMAG5273 requires 4 bytes of data to be read before generating a CRC byte" (E2E 1121557) is the minimum, not the period. A hypothesis from four reports, not a TI statement, and a ten-minute bench test.

So the supported form is `I2C_RD = 00`, 3-byte frames, ≤ 4 data bytes per block (12h–15h + CRC, then 16h–19h + CRC: ~150 bit-times, 375 µs at 400 kHz, 150 µs at 1 MHz), and the cheap form to test is one 7-byte read from 12h (X Y Z + CRC, ~90 bit-times, 225 µs at 400 kHz) with T_CH_EN = 0, which is what 1357408 shows working. 3-byte mode is immune to the neighbour's-address mechanism by construction, since each read follows the device's own register-pointer write; in an eight-sensor round-robin in 1-byte mode the neighbour's address is always the last thing on the bus before a read, exactly the condition Rev C warns about and 1592349 measured. Bench tests, an evening's work: **A**, 7-byte 3-byte-mode reads from 12h, CRC_EN = 1, T off, all eight sensors round-robin, ≥ 5000 cycles, counting mismatches of CRC-8 over the six data bytes; **B**, the same with a 10-byte read, logging where the CRC byte lands with XYZ and with XYZ + T; **C** (only if 1-byte mode is wanted for its shorter frame), 1-byte 16-bit mode, CRC_EN, continuous, round-robin, ≥ 5000 cycles, and for every mismatch check whether prepending the previous transaction's address byte reproduces the received CRC. Zero mismatches: round-robin 1-byte CRC is usable. All mismatches explained by the previous address: deterministic, and the driver may accept either candidate (false-accept 2/256 instead of 1/256). Otherwise 3-byte mode. The `crc8_tmag()` above is right for both modes; only the input differs (the data bytes in 3-byte mode, `{addr_rd, data…}` in 1-byte mode, and in test C also `{prev_addr, addr_rd, data…}`); log `plain_ok / prev_ok / neither` per sensor so a rule, if any, falls out of the counts:

```c
// per sensor s, per cycle: returns 0 plain ok, 1 previous-address ok, 2 neither
static uint8_t prev_addr = 0;                 // last address byte on this bus
int crc_probe(int s, int one_byte_mode) {
    uint8_t buf[11]; int n = one_byte_mode ? 8 : 7;   // XYZ+status+crc | XYZ+crc
    if (!one_byte_mode) i2c_write(addr_wr[s], (uint8_t[]){0x12}, 1);
    i2c_read(addr_rd[s], buf, n);
    uint8_t rx = buf[n - 1], in[10]; int k = 0;
    if (one_byte_mode) in[k++] = addr_rd[s];
    for (int i = 0; i < n - 1; i++) in[k++] = buf[i];
    int r = (crc8_tmag(in, k) == rx) ? 0 : 2;
    if (r == 2 && one_byte_mode) {            // try the previous-transaction address
        uint8_t in2[11]; in2[0] = prev_addr; for (int i = 0; i < k; i++) in2[i + 1] = in[i];
        if (crc8_tmag(in2, k + 1) == rx) r = 1;
    }
    prev_addr = addr_rd[s];
    return r;
}
```

Until the bench says otherwise CRC stays off, and T_CH_EN stays 0 for a second reason: if the enabled-channel hypothesis holds, temperature on moves the CRC from byte 7 to byte 9 and a 7-byte read silently returns CONV_STATUS in the CRC slot.

### 2.4 Conversion timing and the 100 Hz budget

Per-channel conversion is 25 µs at 1×, the first channel 50 µs (DS §5.11 note 2); at 32× each enabled channel costs 800 µs, the first 825 µs (note 3). Rates (DS Table 6-4):

| CONV_AVG | N | 1 axis | 2 axes | 3 axes | 3-axis period |
|---|---|---|---|---|---|
| 000 | 1 | 20.0 kSPS | 13.3 | 10.0 | 100 µs |
| 001 | 2 | 13.3 | 8.0 | 5.7 | 175 µs |
| 010 | 4 | 8.0 | 4.4 | 3.1 | 325 µs |
| 011 | 8 | 4.4 | 2.4 | 1.6 | 625 µs |
| 100 | 16 | 2.4 | 1.2 | 0.8 | 1.225 ms |
| 101 | 32 | 1.2 | 0.6 | 0.4 | 2.425 ms |

Temperature: at 1× "the conversion time doesn't change with the T_CH_EN bit setting" (a temperature sample is taken at the start of every conversion regardless and used by MAG_TEMPCO, DS §7.1.3.1). With averaging the datasheet only says each channel costs 32 × 25 µs; the firmware header's 3.2 ms with T on at 32× (825 + 3 × 800) is consistent but *(inferred)*, and the temperature noise only halving from 1× to 32× hints the T channel is not simply averaged 32 times. The SET_COUNT increment rate at 32× with and without T_CH_EN answers it in a minute. Other timings (DS §5.11): power-up to standby 270 µs; sleep to standby 50 µs, with the first address after sleep *not* acknowledged; standby to continuous 70 µs; INT pulse 10 µs.

**Frame budget** *(inferred)*: at 32× XYZ a sensor produces a result every 2.425 ms, four per 10 ms frame. A 7-byte 1-byte-mode read is ~75 bit-times: 190 µs at 400 kHz, 75 µs at 1 MHz. Eight sensors × 4 reads = 6.1 ms of blocking bus per frame at 400 kHz, 2.4 ms at 1 MHz. The 3-byte 10-byte-with-CRC read is ~300 µs at 400 kHz, so 8 × 4 reads (9.6 ms) do not fit there; 8 × 2 (4.8 ms) do at a √2 noise cost; at 1 MHz 8 × 4 is 3.8 ms.

**What the loop measures against that** (checked in the tree at commit 3a37874, `MagArray.cpp`, `JumperlOS.cpp`, `ST7789.cpp`): the loop delivers 2.9 reads per frame, not 4, and the frame is 12–13 ms, not 10. `MagArray::service` runs every scheduler pass that is due (`MAG_SAMPLE_PERIOD_US` 2500, scheduled from the run *start*, so a pass is 2.5 ms plus the service's own time), reads all eight sensors blocking (~1.6 ms with 1-byte reads, `wireless-probe-sensing.md`), and publishes a frame only `if (nowUs − lastFrameUs >= MAG_FRAME_PERIOD_US) { lastFrameUs = nowUs; … }`: the frame ends at the first pass *after* 10 ms have elapsed and re-stamps from that moment, so its period is 10 ms plus the remainder of a pass. A pass is the read plus whatever else is due, and a MagView band (2.4 ms: 3600 pixels byte-swapped in place, then a `transferDMA` that waits in `dma_wait`) is due every 3 ms, so it is in nearly every pass; ~4.5 ms per pass and 10 + U(0, 4.5) ms averages 12.3 ms ≈ 80 Hz against the measured 76 Hz, and 30-row bands (4.8 ms) give the 65 Hz also seen. `MAG_FRAME_PERIOD_US 10000` is a floor, not a period, and never could produce 100 Hz. Two consequences: the per-frame noise ceiling below is not reached because the LCD holds the loop ~45 % of the time, and the tracker's dt is measured when `MagLocator` runs, not when the data was taken, so scheduling latency of up to a pass leaks into Q. Fixes, in order: a phase-locked stamp (`lastFrameUs += MAG_FRAME_PERIOD_US`, resynchronised if more than one period behind) for the mean; the frame carrying the mean `micros()` of its reads and their count, so the tracker's dt is the data's and a frame of one read is weighted as one; and the reads or the LCD off the V5F for the jitter. The code already names the cure for the LCD (`MagView.h`: "sample from the second core… so that the display never delays a read"): the V3F can push bands from the framebuffer in shared SRAM, which both cores reach (CH32H417 DS V1.8 §1.4.2; `CH32H4Mutex` exists in the core), or the core's SPI gains a start-and-poll DMA call, since the present `transfer` has only the waiting form; swapping bytes at draw time removes the 3600-pixel loop either way. The 1.6 ms of blocking `Wire` per pass is likewise a V3F or I²C-DMA job on V6, and with the reads on the V3F the frame is stamped by the reader itself.

**The noise ceiling** *(inferred)*: the chain digitises one sample per axis per 25 µs, so with three axes at most ~133 samples per axis land in a 10 ms frame, fewer at low CONV_AVG because the 50 µs first-channel overhead is paid per result. 32× on-chip × 4 reads = 128 samples per frame is at that ceiling: 125 µT/√128 ≈ 11 µT on X/Y, which is exactly the 0.011 mT/axis measured. No register setting lowers the per-frame noise further. Reading a sensor more often than every 2.425 ms returns duplicate results; SET_COUNT tells the two apart.

### 2.5 Noise, mode, range, sensitivity, offset, tempco

RMS noise per axis at 25 °C (DS §5.7 x1, §5.8 x2); only 1× and 32× are specified, the rows between are σ₁/√N *(inferred)*:

| Part | Axis | Mode | 1× | 2× | 4× | 8× | 16× | 32× (spec) | 32× by √N |
|---|---|---|---|---|---|---|---|---|---|
| x1 | X, Y | LP | 125 µT | 88 | 63 | 44 | 31 | 22 µT | 22.1 |
| x1 | X, Y | LN | 110 | 78 | 55 | 39 | 28 | 22 | 19.4 |
| x1 | Z | LP | 68 | 48 | 34 | 24 | 17 | 11 | 12.0 |
| x1 | Z | LN | 66 | 47 | 33 | 23 | 17 | 9 | 11.7 |
| x2 | X, Y | LP | 147 | 104 | 74 | 52 | 37 | 24 | 26.0 |
| x2 | X, Y | LN | 145 | 103 | 73 | 51 | 36 | 24 | 25.6 |
| x2 | Z | LP | 89 | 63 | 45 | 31 | 22 | 15 | 15.7 |
| x2 | Z | LN | 88 | 62 | 44 | 31 | 22 | 15 | 15.6 |

Averaging follows √N closely. The vertical X/Y elements are about twice as noisy as the horizontal Z element (DS §7.1.3.3 says so itself). Low-noise mode buys nothing on X/Y at 32× and ~18 % on Z (9 vs 11 µT) for 30 % more current; self-heating at 3.3 V is ≈ 1.6 °C in LN, 1.2 in LP (9.9 mW × 162 °C/W, DS §5.4, *inferred*). Table 6-3 annotates the wider range with "Better SNR performance", unexplained; the bench measurement (`wireless-probe-sensing.md` §6.1, no magnet, 32×, rms over all eight sensors) read 17.7 / 17.2 / 9.9 µT on X / Y / Z at ±80 mT and 17.4 / 17.2 / 8.7 µT at ±40: X and Y unchanged, Z 14 % worse at the wide range, about half a converter LSB (2.4 µT at ±80) added in quadrature. So the noise is the Hall element's, not the converter's step, on X/Y outright and on Z very nearly; the wide range is not "better SNR" but costs almost nothing.

Sensitivity (DS §5.7): 820 LSB/mT (±40) / 410 (±80); error ±5 % typ, **±20 % max** at 25 °C; drift ±5 % typ over −40..125 °C; linearity ±0.1 %; axis mismatch X–Y ±0.5 %, Y–Z and X–Z ±1.0 %, mismatch drift ±5 % / ±15 % over temperature. Offset ±300 µT typ, ±1000 max; offset drift ±3 µT/°C typ, ±10 max. Cross-axis sensitivity and hysteresis "are insignificant" (DS §7.1.5), no number given. TI targets Cpk > 1.67, so ±5 typ / ±20 max implies σ ≈ 3 % on sensitivity (E2E 1591522). No lifetime-drift data exists for the TMAG5273; TI expects it to match the TMAG5173-Q1 (E2E 1590481, *verified*), whose datasheet specifies it (`ds_tmag5173-q1.txt`, *verified*): A1 sensitivity lifetime drift ±1.0 % typ / ±3.74 % max, offset ±100 µT; A2 ±1.5 % / ±4.9 %, ±100 µT, all at 25 °C. The "1000 h at 125 °C" condition is not in that datasheet and stays *(recalled)* from E2E 1567765. The datasheet's error equations: Error_25C = √((B·SENSER)² + Boff² + NRMS²)/B (Eq 19), after room-temperature calibration Error_Temp = √((B·SENSDR)² + Boff_DR² + NRMS_Temp²)/B (Eq 20); TI's worked example (E2E 1402689) uses B = 30 mT, SENSDR 0.05, Boff_DR = 3 µT/°C × 25 °C = 75 µT, NRMS 147 µT.

What that means at our amplitudes: a ±1 % X–Z mismatch on a 30 mT near-field reading is 300 µT, six times the unexplained 50 µT error, so `magcal` should fit three gains per sensor, not one. A ±5 °C room swing at 3 µT/°C is ±15 µT of offset drift per axis, at the max 10 µT/°C it is 50 µT, the size of the whole far-field signal; the bench log records 0.1 mT of drift in ten minutes. The baseline has to be refreshed while the probe is away, not only at boot.

**MAG_TEMPCO.** It scales the magnetic result by 0.12 %/°C (NdFeB) or 0.2 %/°C (ferrite) of the *die* temperature, to cancel the Br drift of a magnet at the same temperature as the sensor (DS §5.9, §7.1.2; Br tempcos from SLYA059B: NdFeB −0.12 %/°C, SmCo −0.04, AlNiCo −0.02, ferrite −0.2). The probe's magnet is in a hand and the die is on the PCB, and the fit solves the moment magnitude every frame anyway, so Br drift is absorbed for free. The NdFeB setting adds a die-temperature-tracking gain error and removes nothing. The field-notes summary of TI's app notes says 0.12 %/°C; the datasheet reading wins here: **set 00**. (Adafruit's driver sets NdFeB; the Linux IIO and SparkFun drivers leave it off.)

**Angle engine, gain and offset registers.** ANGLE_EN picks two axes; the CORDIC returns 0–360° in 1/16° steps (±0.5° X–Y, ±1° otherwise at 32×). The gain register scales one of the two channels and the offset registers subtract up to ±128 × 19.5 µT from each; the datasheet describes them only as part of the angle feature and never says the raw results are corrected. Scott Bryson notes the datasheet's gain formula is backwards: multiply the ratio by 256, not divide (E2E 1467768, *verified*). Two axes, 8 bits, RAM: none of it helps a three-axis fit. Leave ANGLE_EN = 0 and keep gain and offset in firmware.

### 2.6 Interrupts, thresholds, power modes, triggered sampling

Threshold LSB is 312.5 µT at ±40 mT, 625 at ±80 (DS §8.1.5), compared against the raw offset-bearing reading, so with ±300 µT of typical offset plus Earth's 50 µT a reliable "magnet came near" band starts around 0.6–1 mT, the field at the array's edge and not the 0.1–0.3 mT far field. Wake-up-and-sleep mode (DS §6.4.3): sleep for SLEEPTIME, convert, compare, assert INT on a crossing; 160–240 µA average at 1 ms, 1.2 µA at 5 s. A user and then TI's Alicia Rosenberger both reproduced W&S threshold interrupts stopping after minutes to hours at 100/400 kHz with INT_ER set; TI's theory was a race between the W&S transition and I²C wake-up, its remedy a 1 MHz bus or MCU-driven sleep/wake (E2E 1171980). A latched interrupt is cleared by *any* I²C traffic on the bus, so INT cannot flag a particular device (Rosenberger, E2E 1618505, *verified*: "one of the conditions to clear the interrupt is whether any I2C communication is detected on the I2C bus, regardless of which device is being addressed"), and "there is not a way for the device to stop signaling interrupts after the first initial crossing when the magnetic field remains above the set threshold" (same thread). Verdict: not a far-detection mechanism; at most a holstered-probe power trigger, and the W&S stall TI reproduced but never root-caused is one more reason. Distance at range has to come from the fit and its residual (§4).

Result-ready on INT (RSLT_INT 1, INT_MODE 001, INT_STATE 1: register A4h, which is TI's own tamper example) is useful: eight open-drain INT pins wired-OR on one MCU input give a ready pulse instead of polling. SCL-as-interrupt is out on a shared bus (DS §6.3.3).

Power modes (DS §6.4, Table 6-5, §5.5): standby/trigger 0.45 mA, one conversion per trigger; sleep 5 nA, keeps configuration and the assigned address, loses results, 50 µs to wake; continuous 2.3 / 3.0 mA; W&S 1–240 µA. **Standby/trigger is the interesting one for an array.** A general-call write with the trigger bit set starts a conversion in every sensor at the same instant (DS §6.5.1.3.2, §7.1.3.2), or TRIGGER_MODE = 1 lets one GPIO on the shared INT net do it with no bus traffic; redundant triggers are ignored and a running conversion completes first (DS §8.1.2). Today the eight sensors free-run and are read up to ~2 ms apart, a real smear at hand speed for a fit that assumes one instant; SBAA539 and SBAA540 both insist on "a deterministic measurement scheme … using the integrated trigger modes". Sketch:

```
i2c_write(0x00, {0x80 | 0x18, 0x00});   // general call + trigger: all sensors convert now
wait_us(2500);                           // 32x, 3 axes (Linux driver: 2425 us max)
for s in sensors: read_1byte_16bit(s);   // X,Y,Z,CONV_STATUS, ~75 us each at 1 MHz
```

Check RESULT_STATUS and that SET_COUNT advanced. No source ran this with eight TMAG5273s, and the CRC-contamination report (E2E 1592349) means it should be bench-verified with CRC off first. Two further robustness notes from the forum, now on verbatim ground: parts occasionally freeze in continuous 32× LN mode with "the RESULT_STATUS … high and the SET_COUNT … frozen and does not increment", no error flags, and only a power cycle recovers (E2E 1519712; Rosenberger: "not typical, so there is not any documentation on it"); there is no software reset, "there is not a reset routine for the device beyond adding a load switch and power cycling the device" (Rosenberger, E2E 1357951), which is one more reason to keep the per-sensor supply switches. OSC_ER after power-up comes from a supply glitch between 0.35 and 1.5 V, was seen on "2 out of 20 units", and can be ignored unless sleep modes are used (E2E 1498216). Bus pins at 3.3 V with VCC off: "I checked with design and was told that the device should be able to handle this condition" (Bryson, E2E 1394437). Clear POR after power-up and log DEVICE_STATUS in the recovery path, since a sensor pulling 3 mA through a GPIO's on-resistance is the sort of thing that trips VCC_UV_ER (1.5 V typ / 1.39 min).

### 2.7 What TI's application notes add

- **SLYA051B, Linear Hall-Effect Sensor Array Design** *(verified)*: DRV5055 but "the observations and principles … apply equally to the DRV5057, TMAG5170 and TMAG5173-Q1". Spacing rule: "the maximum appropriate sensor spacing is roughly the magnet length … it is recommended to have overlap between sensors"; five sensors at ~20 mm for ~100 mm with a 6.3 × 22.2 mm N42 at 8 mm gap. Their selection algorithm picks the sensor with the greatest |output| and then its neighbour by the sign. Measured error sources: per-device sensitivity spread, soldering alignment, and **one uneven gap produced a visible up-tick at 55 mm**; the fix was even spacing.
- **SLYA059B, Magnet Selection for Linear Position** *(verified)*: "the maximum sensing range is typically at least 2x the diameter of the magnet"; aim for a field that "neither saturates the input of the sensor nor becomes distorted due to close range of the magnet"; thickness-to-diameter ratio limits corner distortion. Its linearisation `Position = tan(γ·atan((α·Bz − β)/Bx)) + φ·Bx·(thickness + airgap)/2` with fitted α, β, γ, φ (e.g. 0.791, 16.3, 0.4104, 0.448 for a 10 × 4 mm magnet at 10 mm) reaches ~10 µm residual over ~2 diameters.
- **TIDA-060045 / TIDUF78, Quad 3D Hall linear position** *(verified)*, the closest TI publication to this problem: four TMAG5170A1 at 25 mm pitch, an N45 25 × 3 mm disc at 15.2 mm gap, 4 kHz frames, all four triggered simultaneously by ALERT; "linear position accuracy typically ±0.15 mm over 100 mm range", static σ 0.053 mm; each sensor gives 8.6 ENOB over 2.5 cm and the array 10.6 over 10 cm. Noise at 1×: Z 0.059 mT rms at ±50 mT, X 0.094 at ±25. Algorithm: largest |Z| picks the sensor (with hysteresis on handover), "if none of the Z-axis magnitude exceeds a minimum field strength, the moving magnet is out of range", angle = atan2(X, Z), position from angle plus sensor index plus a |X| correction. Calibration spread on nominally identical parts: Z offsets 14.6 / 14.3 / 14.0 / 13.8 mT, gains 0.94 / 0.93 / 0.94 / 0.94.
- **SBAA540A, SLYA070A** *(verified)*: adjacent 3D Hall sensors "a few millimeters up to 10's of millimeters" apart; find the sensor nearest each mover by the peak Z, then gain/offset-correct; an N52 10 mm magnet gives < 0.1° error over ±13 mm with only gain and offset correction; fan-out needs bus buffers for large arrays.
- **SBAA539A** *(verified)*: "Amplitude mismatch is the dominant factor which influences accuracy"; averaging by √N; "use a deterministic measurement scheme … using the integrated trigger modes".
- **SLIA086A**: multiply RMS noise by 6.6 for peak-to-peak.

Mapped to the array: the pitch-vs-magnet rules (pitch ≈ magnet length with overlap; range ≈ 2 diameters) are for single-sensor methods that a 5 mm probe magnet at 20 mm pitch is well past; the dipole fit is what rescues it and is why fixes drop at the edges. Per-sensor gain and offset calibration is mandatory in every TI design, even with the tighter TMAG5170.

### 2.8 Community lessons on the bus

One set of 4.7 kΩ pull-ups per bus and none on a switched sensor's side: with one sensor powered the bus of a 16-sensor array sagged to ~2 V through the unpowered breakouts' pull-ups (E2E 1223356); the pin's sink rating is 2 mA (E2E 1624039). Power-cycling is fine and re-configuration after any reset is harmless (E2E 1433746, 1366574). A GPIO-sequenced supply with an RC delay per sensor and general-call configuration is a published pattern (github.com/TuYuxiao/TMAG5273). Open-source driver defaults, for comparison: SparkFun v2 leaves averaging at 1× and both ranges at 80 mT; Adafruit sets 32×, LN, wide range, NdFeB tempco, ANGLE_EN XY and result interrupt; Linux IIO sets 32×, continuous, XYZ + T, low range, and notes "max conversion time is 2425 us in 32x averaging mode"; Zephyr defaults to 2× and computes CRC as `crc8_ccitt(0xFF, data, 4)` over standard reads padded to 4-byte blocks.

### 2.9 Alternatives, should the part change

| Part | Interface / addressing | Range | Noise at best averaging | Sensitivity error | Package |
|---|---|---|---|---|---|
| TMAG5273 | I²C 1 MHz; 4 factory addresses, RAM re-address | ±40/80 (x1), ±133/266 mT (x2) | XY 22, Z 9–11 µT | ±5 % typ / ±20 % max | SOT-23-6 |
| TMAG5173-Q1 | I²C, same addresses, RAM | same | XY 15, Z 9.4 µT (LN) | **±0.4 % typ / ±2.5 % max**, lifetime drift specified, 1-byte CRC works | SOT-23-6, pin-compatible |
| TMAG5170 | SPI 10 MHz + CRC, ALERT sync | ±25/50/100 (A1) | XY 24, Z 11 µT | ±0.5 % / ±2.5 % | VSSOP-8 |
| TMAG3001 | I²C, **ADDR pin → 0x34–0x37** | ±40/80/120/240 mT | XY 16, Z 9 µT | ±1.8 % / ±7.7 % | DSBGA 0.83 × 1.32 |
| ALS31313 | I²C, 16 addresses by divider, 127 via EEPROM | ±50/100/200 mT | XY 100, Z 37.5 µT | ±4 % | TSSOP-8 |
| MLX90393 | I²C/SPI, 4 codes × 2 pins = 16 | 5–50 mT | charts only | | QFN-16 |
| MMC5983MA | I²C fixed 0x30 / SPI 10 MHz (a CS per sensor: the addressing solved) | **±0.8 mT** | 0.04 µT at 50 Hz, 0.12 at 1 kHz | | LGA-16 3 × 3 |
| TMAG3001 (TI, 2025) | I²C 1 MHz + CRC; **ADDR pin → 0x34-0x37**, and a RAM re-address like the 5273 | ±40/80 (A1), ±120/240 mT (A2) | XY 16, Z ~9 µT at 32×, but a 32× 4-channel conversion is 0.8 ms (5273: 2.4) - three times the conversions a frame | ±1.8 % typ / ±7.7 % max | DSBGA-6 0.83 × 1.32 |
| TLI493D-A2B6 (Infineon; LCSC C1559664, $0.40-0.74) | I²C; default 0x35, two order-code addresses | ±160 mT full / ±100 short | 12-bit: 130 / 65 µT per LSB; 0.1 mT rms typ, 0.25 max per conversion at up to 7.5 kHz | | TSOP-6 |
| AK09918C (AKM Hall compass; LCSC C2655004, $0.20-0.40) | I²C **fixed 0x0C**; 1.65-1.95 V supply only | **±4.9 mT** (overflow flag) | 0.15 µT/LSB; noise not specified (class ~0.2 µT); one 8.2 ms conversion, 100 Hz max | | WLCSP-4 0.8 mm |
| QMC6309 (QST AMR; LCSC C5439871, $0.18-0.36) | I²C **fixed 0x7C** ("contact factory for more"); 2.5-3.6 V | **±3.2 mT** (±1.6 / ±0.8 selectable) | 16-bit, 0.25 µT at OSR 8/8, 200 Hz | | WLCSP-4 0.8 mm |
| BMM350 (Bosch) | I²C (ADSEL pin: 2 addresses) / I3C; 1.72-1.98 V supply only | **±2 mT** | 0.19 µT XY / 0.45 Z at 100 Hz; 400 Hz max | | WLCSP-8 1.28 mm |
| MLX90393 (Melexis Hall) | I²C / SPI; **A0/A1 pins × 4 order codes = 16 addresses** | 5-50 mT programmable (saturation 50 mT) | 0.16-3.2 µT/LSB; ~1-5 µT rms by OSR/filter (0.26-66 ms conversions) | | QFN-16 3 × 3 |
| ALS31313 (Allegro) | I²C 1 MHz; 16 addresses by resistor, 127 by EEPROM | ±50/100/200 mT | 12-bit 4/2/1 LSB/G: 100 µT rms per sample (500 G part) at 2 kHz | ±1 % typ | TSSOP-8 |
| IIS2MDC (ST; LCSC C2655002, $2.74-4.28) | I²C fixed 0x1E / **SPI** | ±4.9 mT | 0.3 µT rms (LPF on) at 100 Hz max (150 Hz LP) | ±7 % | LGA-12 2 × 2 |
| LIS3MDL (ST; LCSC C478483, $3.27-5.24) | I²C 2 by SA1 / **SPI** | ±0.4-1.6 mT | 0.32 µT rms X/Y, 0.41 Z at UHP 155 Hz; 1 kHz in LP | | LGA-12 2 × 2 |
| QMC5883P (QST AMR; LCSC C2847467, $0.85-1.57) | I²C fixed 0x2C; 2.5-3.6 V | ±3 mT (±0.2/0.8/1.2 selectable) | 0.2 µT X/Y, 0.3 Z; **ODR to 1.5 kHz** | linearity 0.5 %FS, tempco 0.05 %/°C | LGA-16 3 × 3 |
| BMM150 (Bosch; LCSC C171681, $0.69-1.30) | I²C 2 addresses / SPI | ±1.3 mT XY, ±2.5 Z | 0.3 µT at 20 Hz (high-accuracy preset), 1.0 µT at 300 Hz forced | | WLCSP-12 1.6 mm |
| KTH5772/5791 (Conntek; LCSC) | UART / I²C | joystick-specific: reports a processed two-axis position, not the field | - | - | QFN-16 |
| TMAG3001A2 on LCSC: C31115089, $0.46-0.76, 446 in stock (2026-09-20). MMC5983MA: C404329, $1.40-2.28. | | | | | |
| MMC5633NJL (Memsic, Rev A 2022) | the MMC5603NJ with the I3C documented (MIPI 1.0 slave: ENTDAA, SETDASA, IBI, SETXTIME sync); PID fixed 0x04A2_0000_F000 on every unit, so ENTDAA cannot separate identical parts on one bus; I3C mode needs VIO 1.5-1.8 V; body 0.85 mm, same 0.4 mm ball map | same | same | same | WLP-4 0.85 × 0.85 mm |
| MMC5603NJ (LCSC C404328, $0.15-0.29) | I²C fixed 0x30 (8 factory variants by order) / I3C, no DAA documented | **±3 mT**, linear to ±1.5, "disturbing field" 3.2 mT, per-axis continuous limit 1.6 mT | 0.15-0.2 µT per 6.6 ms measurement (BW00, 75 Hz), 0.4 µT at 1.2 ms | ±5 %, 5 % over -40..85 °C; null tempco 0.02-0.1 µT/°C; SET/RESET on chip | WLP-4 0.8 × 0.8 mm |

The only drop-in that fixes the TMAG5273's real weakness (±20 % max gain, unspecified lifetime drift) is the TMAG5173-Q1; TMAG3001 fixes addressing; TMAG5170 fixes both plus synchronisation at the cost of chip selects. Compass-class magnetometers (MMC5983MA, MMC5603NJ, LIS3MDL, AK09940A) are a hundred times quieter but saturate below ~2–3 mT and are wrong for a 10–30 mT near field (checked 2026-09-20 for the MMC5603NJ against the bench: the night's probe at rest in row 35 puts 1.5 mT on the nearest sensor and a tip straight over a sensor at the learned 10.8 mm surface would put 6.7 mT on it - past the ±3 mT range and the 3.2 mT disturbing field; with a magnet 8-14× weaker (m ≈ 300-500 mT·mm³) the nearest sensor stays under 0.8 mT and the farthest still sees 9-14 mG against 1.5 mG of noise, 5-10× the margin the TMAG array has today). The survey of 2026-09-20 (rows above) sorts into three: Hall parts that keep the magnet and the noise (TMAG5173-Q1 the drop-in with ±2.5 % max gain; TMAG3001 the one with an address pin and 3× the conversions; TLI493D and ALS31313 coarser), compass-class parts that need the magnet 2-14× weaker and buy 30-100× the noise (IIS2MDC and MMC5983MA with SPI for the addressing, AK09918C/QMC6309/MMC5633 cheapest but fixed I²C addresses, BMM350/AK09940A quietest but 1.8 V only), and MLX90393 between them (50 mT, 16 pin-set addresses, 3-10× quieter, slower). Ranked by price, accuracy and speed alone (Kevin's criteria, 2026-09-20; footprint and addressing waived, the magnet free to change), the AMR compass parts win outright: the MMC5603NJ/5633NJL at $0.15-0.29 gives 15-20k of usable range over per-frame noise against the TMAG5273's ~1.8k, the QMC5883P the same at 1.5 kHz for a dollar, the MMC5983MA 13-20k with SPI at $1.40-2.28; the Hall parts are 5-10× behind per dollar and are only worth keeping if the magnet must stay strong. For eight sensors a board (Kevin, 2026-09-20: "low cost, there are 8 of them"): 8 × MMC5603NJ = $1.35 at the 500 break ($1.24 at 2,500) plus two 74HC4051s (~$0.10) switching SDA and SCL to give the fixed-address parts a bus each, against 8 × TMAG5273 ≈ $3.5-4.5 today - cheaper and ~10× more accurate, with the magnet at a quarter of its present moment; a mixed array (Hall near, AMR far) remains an option for V6.

### 2.10 Recommended settings, and what the driver should change

Register set for x1 parts, eight on one bus, 100 Hz frames:

| Register | Value | Why |
|---|---|---|
| DEVICE_CONFIG_1 | CRC_EN 0, MAG_TEMPCO 00, CONV_AVG 101 (32×), I2C_RD 01 | tempco off (§2.5); 32× is at the frame's noise ceiling (§2.4); 1-byte reads unless the §2.3 CRC test passes |
| DEVICE_CONFIG_2 | THR_HYST 0, LP_LN 1, I2C_GLITCH_FILTER 0 (= on), OPERATING_MODE 10 now, 00 with triggered sampling | LN gives Z 9 vs 11 µT for nothing on X/Y; keep unless the GPIO supplies run warm |
| SENSOR_CONFIG_1 | MAG_CH_EN 0111 | XYZ |
| SENSOR_CONFIG_2 | ANGLE_EN 0; X_Y_RANGE = Z_RANGE = 1 (±80 mT) | 2.44 µT/LSB is 9× under the X/Y noise and ~4× under Z's; measured no penalty on X/Y and 14 % on Z (§2.5); sensors stop being dropped at 38 mT when the probe is close, so the fit keeps its nearest, most informative sensor |
| T_CONFIG | T_CH_EN 0 | off until the SET_COUNT test shows its cost; drift is handled by opportunistic re-baselining; and with CRC on, T enabled probably moves the CRC byte (§2.3) |
| INT_CONFIG_1 | A4h on V6 (INT routed, shared); bench: MASK_INTB 1 as now | ready pulse replaces polling |

Driver changes, in the order they pay:

1. **Use the status byte.** Every 1-byte read ends with CONV_STATUS, `tmag5273Read` stores it, and `sampleSensors` never looks. SET_COUNT must advance by 0 or 1 mod 8 between successive reads of one sensor: 0 is a duplicate (do not add it to the frame sum, it makes `samplesPerFrame` lie), > 1 means results were missed (count it), anything else is a misaligned frame. RESULT_STATUS must be 1; DIAG_STATUS flags supply, clock and INT faults. Zero bus cost.

```c
uint8_t cnt = r.status >> 5;
uint8_t step = (uint8_t)((cnt - s.lastSetCount) & 7);
s.lastSetCount = cnt;
if ((r.status & 0x01) == 0 || (r.status & 0x02)) { s.badFrames++; continue; }
if (step == 0) continue;                       // same conversion as last time
if (step > 1) s.skipped += step - 1;
```

2. **Drop the per-axis median of three** once (1) is in and a per-axis 3σ range check on |B − lastGood| over 2.5 ms is added: the median compensates for undetected bus corruption at one frame of latency and rounds off fast motion.
3. **MAG_TEMPCO 00** (was NdFeB; switched off on 2026-09-18 from this finding).
4. **Synchronise reads with conversions**: `MAG_SAMPLE_PERIOD_US 2500` against a 2425 µs conversion drifts through the phase, misses ~1 result in 32 and yields duplicates when jitter brings two reads closer than 2.425 ms. Either the SET_COUNT rule or triggered sampling from a timer (§2.6), which also makes the eight readings simultaneous.
5. **±80 mT** instead of dropping saturated sensors at 38 mT.
6. **Three gains per sensor in `magcal`**, and a 3 × 3 per sensor if the residual stays structured after a calibration with the magnet at 15–25 mm.
7. **A live baseline** (today: 64 frames at boot): a slow per-axis IIR updated only while the presence test says no magnet (§4.6), with the existing dipole-pollution check before any re-zero.
8. **Per-axis weights in the LM fit**: σ_xy ≈ 22 µT and σ_z ≈ 9 µT mean Z residuals deserve ~6× the weight of X/Y (1/σ²), divided by the number of reads averaged for that sensor in the frame.
9. **1 MHz on V6** (short traces, ~1 kΩ pull-ups at 3.3 V, 120 ns rise) cuts the read-out from ~6 ms to ~2.4 ms per frame, or split the array over two of the CH32H417's I²C peripherals.
10. **Hall-element positions** (§2.1) in the V6 footprint-to-position table before `magcal` refines them.
11. **Stamp the frame from the data, not the scheduler** (§2.4): a phase-locked `lastFrameUs`, and the frame carries the mean `micros()` of its reads and their count, so the tracker's dt and per-frame weight are honest. The two locator constants that are per-frame, `MAGLOC_SLOWEST_ALPHA 0.08` and `MAGLOC_BASELINE_DRIFT 0.002`, become α = 1 − exp(−dt/τ) with τ = 0.12 s and 5 s (30 s once §4.6's rule is in), and `MAG_BASELINE_FRAMES 64` a duration.

Bench measurements, each under a minute: SET_COUNT rate at 32× with T_CH_EN 0/1, and with one axis versus three (it also tells whether a channel's 32 samples are contiguous or interleaved, which §2.11 needs); no-magnet standard deviation at ±40 vs ±80 and LP vs LN (already done for range); the §2.3 CRC tests A and B in 3-byte mode across the round-robin; how many distinct SET_COUNT values land per frame today (expect 4; measured 2.9 reads).

### 2.11 The board's own fields

**Status of the evidence.** `JumperlessV6/MainBoard/MainBoard.kicad_pcb` is placement-only: 1000 WS2812B footprints (600 × 1313 in the 16 × 30 block, 400 × 1010 in a second experiment), twelve CH446Q, the RP2350, and **zero routed segments, no TPS65131, no TMAG5273** (*verified*, parsed 2026-09-17). Every layout number below is from the routed V5r7 board (`Untitled/Untitled.kicad_pcb`, *verified*) scaled to V6's LED count: "V5 routing, scaled". Nothing has been measured; the protocol at the end settles it.

**What V6 inherits from the V5 routing.** Four-layer 1.6 mm stack, the same in both files: F.Cu 35 µm, 0.1 mm FR4, In1.Cu, 1.24 mm core, In2.Cu, 0.1 mm FR4, B.Cu; GND is a zone on In1/In2/B.Cu under the LED field. The LED **+5V is an end-fed tree**: two 0.3 mm F.Cu trunks cross the full LED width at y ≈ 112.0 and y ≈ 129.8 mm, both fed from the right end, and a 0.2 mm feeder drops from a trunk down every LED column to five LEDs (93 feeders, 1.10 m of 0.2 mm trace). With V5 rows at y = 113.1–123.2 (rows 1–30) and 130.8–141.0 (rows 31–60), the upper trunk is 1.1 mm from the row-1 LEDs and the lower trunk runs **through the 7.6 mm centre gap**, 1.0 mm from row 31. Each trunk feeds ~200 LEDs and carries all of their current at its fed end. VBUS enters at the USB-C and runs as 0.25 mm B.Cu 15 mm above the LED rectangle. The V6 sensors will sit where those trunks run, so a sensor is 1–4 mm from a trunk unless the routing changes.

**Currents** (XL-1010RGBC-WS2812B datasheet, *verified*: I_DD 0.35 mA typ with outputs off, I_OUT 5 mA typ per colour, f_PWM 1.0 kHz typ; the 1313 assumed equal, *unverified*). Full white is 15.35 mA per LED, not the 5050's 60 mA. JumperlOS draws nets at `DEFAULTBRIGHTNESS 50`/255 (*verified*, `LEDs.h`), ≈ 0.196 mean duty; the outputs are PWM'd, so the instantaneous current is 0 or 5 mA per lit channel and only the mean scales with brightness.

| Case | V5 trunk (200 LEDs) | V6 trunk, same topology (500) | V6 board (1000) |
|---|---|---|---|
| quiescent, dark | 70 mA | 175 mA | 350 mA |
| typical frame (20 % of LEDs, one colour, 50/255) | 109 mA | 273 mA | 546 mA |
| all white at 50/255 | 658 mA | 1.65 A | 3.3 A |
| all white at 255 | 3.1 A | 7.7 A | 15 A (not supplyable) |

Quiescent is constant and goes into the baseline; the dark-to-white swing at default brightness is ~1.5 A per V6 trunk.

**Biot–Savart at the sensor plane.** A DC-to-kHz current's plane return spreads across the whole zone, because at low frequency the return minimises resistance, not loop inductance, and only gathers under the trace above a kHz-range crossover (Ott, Johnson and Graham, *recalled*), so a trace over the plane is an isolated wire, B = µ0 I/(2π r), plus a near-uniform µ0 I/(2W) from the spread return; a trace with its return forced directly beneath on the next layer gives B ≈ µ0 I s/(2π r²), s = 0.135 mm here. With µ0/2π = 2 × 10⁻⁷ T·m/A:

| Source | I | 1 mm | 2 mm | 5 mm | 10 mm | 20 mm |
|---|---|---|---|---|---|---|
| V5 trunk, all white @ 50 | 0.66 A | 132 µT | 66 | 26 | 13 | 6.6 |
| V6 trunk, same topology, all white @ 50 | 1.65 A | 329 | 165 | 66 | 33 | 16 |
| V6 trunk, typical frame | 0.27 A | 55 | 27 | 11 | 5.5 | 2.7 |
| one 5-LED feeder, all white @ 50 | 16 mA | 3.3 | 1.6 | | | |
| cursor patch, ~10 LEDs at 0xA0 white | 94 mA | | | 6.3 µT at 3 mm | | |
| V6 trunk with stacked return, 1.65 A | | 45 | 11 | 1.8 | 0.45 | |
| plane return, 1.5 A over 100 mm | | ~9 µT everywhere | | | | |
| VBUS 3 A, 15–20 mm from the top rail row | | | | | 40 | 30 |

Against the tracker's numbers (11 µT/axis frame noise, 50 µT repeatable error, 100–300 µT far-field signal): an end-fed V6 trunk 2 mm from a sensor moves it by up to 165 µT between a dark and a white frame, more than the whole far-field signal and three times the unexplained error; at 10 mm it is still 33 µT. Feeders are harmless, and the cursor patch is 6 µT at 3 mm but *correlated with the probe position*, so it biases rather than blurs. The size is set almost entirely by where the trunks run. Putting the sensors on B.Cu buys only the 1.6 mm board thickness; the GND zones in between do nothing for DC fields.

**LED PWM ripple.** The 1010's PWM is 1.0 kHz and the TMAG5273 at 32× averages 800 µs per channel (0.8 of a period), so one LED's square wave is *not* averaged out: the boxcar catches between 0 and 1 full pulse. It is nothing on a feeder (1.3 µT rms at 2 mm); on a trunk the LEDs' unsynchronised oscillators turn it into random-phase noise, I_pk √(N d (1 − d)): 84 mA rms for 200 LEDs, 133 mA for 500, i.e. 8 and 13 µT rms at 2 mm, 3 and 5 µT at 5 mm *(inferred)*. This is the one LED term no frame prediction removes; it sits just under the sensor noise at 2 mm and vanishes at 5 mm. A 1× conversion (25 µs) would see the full instantaneous swing, a second reason after the §2.4 noise ceiling to stay at 32×.

**The TPS65131 and the other switchers.** f_S = 1250 min / 1380 typ / 1500 max kHz, recommended inductor 4.7 µH (SLVS493E §5, §8.2.2.2, *verified*). Datasheet Eq 5, ΔI_L = V_I (V_POS − V_I)/(L f V_POS), gives 343 mA p-p at V_I 5 V, V_POS +9 V (514 mA at +15 V), load-independent in continuous conduction; the average inductor current is I_out V_POS/V_I, 36 mA at a 20 mA rail load, 360 mA at 200 mA. So each inductor produces a 1.25–1.5 MHz ripple of a few hundred mA and a DC term that follows the ±rail load. Würth ANP047 (*verified*, qualitative) ranks unshielded > semi-shielded > shielded for near-field H, and Coilcraft's FAQ says field interaction is hard to model and recommends measuring the final board; no vendor note gives µT-at-mm for a 4 mm shielded drum. As a dipole, a bare 10-turn 4 mm² coil at 0.3 A is 20 µT at 5 mm and 0.3 µT at 20 mm; an open ferrite core multiplies that by 10–50, a shielded core confines it *(inferred)*. **Aliasing into the 25 µs sampling** *(inferred)*: if a channel's 32 samples are contiguous at 40 kHz (as DS §5.11 note 3 suggests; if interleaved X, Y, Z the rate is 13.3 kHz and the nulls move to multiples of ~417 Hz; the SET_COUNT test in §2.10 tells), ripple at f_S folds to |f_S − n · 40 kHz|, anywhere in 0–20 kHz across the tolerance band; the 32-sample boxcar nulls at multiples of 1.25 kHz with a worst first sidelobe of 0.21 near 1.9 kHz, and the datasheet gives no front-end bandwidth ("noise filters, integrator circuit", §6.1), so between 0.2 and 0.002 of the ripple field reaches the result. At ≥ 25 mm the ripple field is sub-µT before any attenuation, so aliasing is a non-issue *if the placement rule holds*; at 5–10 mm it would show as a beat between the converter and the sample clock, drifting with temperature and load. The same applies to the RP2350 regulator inductor (V5 `L2`, 3.3 µH, 16 mm above the top LED row) and the CH32H417's if it has one. V5's ±9 V comes from LM2660 charge pumps (`U9`), no inductor, so a V5 breakout test says nothing about the TPS65131; that needs its evaluation board or the first V6. Crossbar and relay currents are not a source (a CH446Q switches signals; its supply is milliamps); the ±rail *load* currents are, through the inductors and through the rail strips 7 mm above the sensors.

**User circuits on the breadboard.** A wire or rail carrying I at height h is the same isolated wire: 0.5 A at 8 mm is 12.5 µT, 1 A is 25 µT, partly cancelled by its return. The rail strips have fixed geometry and, on V5, measured currents (two INA219s, `Apps.cpp`, *verified*), so they are a known-input term like the trunks. User wires are not, and they are one reason the tracker keeps a residual gate: a misfit jump while the LED prediction and rail currents are steady is a wire, not the probe. Steel-leaded parts and steel-cored jumpers are a static soft-iron matter for the baseline refresh. The Spring Clip6 contacts are bronze (Fulimei packing list, PO 20260617), non-magnetic.

**Prior art for the known-input correction.** ArduPilot *CompassMot* (`ArduCopter/compassmot.cpp` and the "Advanced Compass Setup" page, *verified*) is the direct precedent: the interference "is linear with current drawn", so with the vehicle held still it low-passes the zero-throttle base and, above 3 A, fits a per-axis vector `motor_compensation = 0.99·motor_compensation − 0.01·(field − base)/current`, stored per ampere and subtracted as K·I; interference under 30 % of the expected field is acceptable, over 60 % means "move the sensor". Zhang et al., Sensors 22(16):6151, 2022 (*verified* abstract, §2) treat aircraft electronics the same way, "quasi-static current … estimated by using the Biot–Savart law" with fixed cable positions, and name the limit: one current regressor per path, as accurate as its current sensor. Our advantage is that the LED currents need no sensor; the firmware computes the frame and can compute the current from it.

**Layout rules for V6** (V6 is unrouted, so routing comes first and the correction takes what routing cannot reach). (1) No LED trunk, VBUS path or rail-supply trace within 10 mm of a sensor; feeders only (5 LEDs, < 4 µT). (2) A trunk that must cross the array carries its GND return as a dedicated trace directly beneath it on In1 (pair field: 11 µT at 2 mm and 1.8 µT at 5 mm for 1.65 A, against 165 and 66 µT for a lone trace); the plane does not do this for DC. (3) Feed the LED field from both ends or the middle so no point of a trunk carries more than half the total. (4) TPS65131, its inductors, input capacitor and diode loop, and the RP2350/CH32 regulator inductors ≥ 25 mm from the sensor rectangle, shielded parts (SLVS493E Table 8-3: TDK VLF4012, Würth 7447789), switching loop under 1 cm². (5) USB-C and the VBUS trunk on the far side of the board from the array, or stacked with their return.

**Known-input correction.** The field is linear in the currents and the geometry is fixed, so per sensor axis `B_meas = B_dipole + B_base + Σ_j K_j I_j`, with I_j the currents the firmware already knows: one per LED chain, `I_chain = Σ_LEDs (I_q + k (R + G + B)/255)`, I_q = 0.35 mA, k = 5 mA (1010 datasheet; measure the 1313), plus the two INA219 rail currents. K is 24 axes × n_currents, calibrated once, CompassMot-style: probe away, show test frames (each chain dark, 25 %, 50 %, 100 % white; each half; each rail loaded), log 1 s per state, least squares. Per frame: `for s in sensors: B[s] -= Bbase[s] + K[s]·I`, before the fit, with the frame's `I` published by the render code when `show()` completes so a fix straddling a latch uses the right frame (on V6 it crosses the link with the ack, §9.5); log `I` with the fix. Expect 10–20 % of the raw effect to remain (LED current tolerance and drift; K itself is exact for fixed geometry). *Do not* blank the LEDs to sample: WS2812-class LEDs hold their PWM until the next frame, so blanking costs a dark frame (14.4 ms per 480-LED chain) plus a re-send at every sample, and pulling ENP/ENN drops the analog rails. Keep dark frames as the `magcal ledcal` diagnostic that measures K and the raw effect. Widen the misfit gate by `BOARD_FIELD_GATE_UT 30` for one frame when the predicted chain current changes by > 200 mA between frames, so a frame change is not read as probe motion.

**Measure before routing** (V5, TMAG5273 breakout, `magcal` logging, 10 s per state, mean shift and rms per axis): breakout in the centre gap over the lower trunk at its fed end (x ≈ 100 mm) and its far end (x ≈ 40 mm), between rows 3 and 4, and on the top rail row 15 mm from the VBUS run; states: LEDs off / all white at 25, 50, 100 % of the cap / normal palette / cursor patch moving; then rails loaded at 0, 100, 500 mA through a jumper crossing the breakout. Separately, on the TPS65131 EVM or the first V6: breakout at 5, 10, 25 mm from the inductor, rail load 0 / 50 / 200 mA, at 1× and 32×, looking for a beat. V5 LED numbers scale to V6 by LED count; the TPS65131 numbers cannot come from V5.

## 3. Prior art: tracking a magnet with a sensor array

### 3.1 The systems

**Planar Hall arrays and permanent magnets.** Schlageter, Besse, Popovic and Kucera (Sens. Actuators A 92, 2001, *abstract*): 16 planar Hall sensors, a 0.2 cm³ magnet, 5 DoF at up to 50 Hz, 14 cm detection range "during 1 hour without calibration", with offset drift, sensitivity mismatch and sensor count as the accuracy limits. Hu, Meng and Mandal (IEEE Trans. Magn. 2007, *abstract*): a linear closed form for the 5-D pose from five or more 3-axis sensors, used to seed a nonlinear fit; Hu et al. 2010 (*abstract*): 15 sensors in one layer of a 0.5 m cube give 1.8 mm average error; Hu et al. 2015 (*verified*): with 5 mm / 5 % noise on 16 sensors "the LM algorithm lapses when the initial guess of location is 15 cm" off, which is why every capsule paper pairs LM with a closed-form or grid initialiser. **Farajidavar, Block and Ghovanloo (EMBC 2012, *verified*)** is the paper closest to our error budget: eight AMI306 3-axis sensors and a 3.1 × 1.5 mm magnet on a 3.75 µm robot. With perfect geometry the simulation reconstructs to 4.17 × 10⁻⁶ mm, so "the major error source was not the optimization algorithms per se, but rather the practical issues involving the sensor calibration and the EMF". They fit sensor positions, orientations, gains, magnetisation and ambient field in one Nelder–Mead problem from robot trajectories; then "within 3 cm from the sensors' plane, the RMS-error was less than 0.5 mm", and beyond 3 cm "increased dramatically". Son, Yim and Sitti (2016, *abstract*): an 8 × 8 single-axis array removes an actuating electromagnet's field by subtracting its dipole model and differentiating twice normal to the array, since a distant source is smooth across the array and a near one is not.

Two open-access Hall-array papers give error against height at comparable pitch. Micromachines 17(3):327, 2026 (*verified*): 40 single-axis DRV5055A2 at 10 mm pitch, a 9 × 5 mm N52, LM; static RMSE 0.98 / 0.76 / 1.62 mm (x/y/z); RMSE at 20 → 70 mm height 1.22, 1.69, 2.33, 2.96, 2.78, 6.23 mm; practical limit ~60 mm "from reduced signal amplitude and signal-to-noise ratio, rather than … instability of the dipole modeling assumption". Sensors 20(20):5728, 2020 (*verified*): 25 WSH202 at 8 mm pitch, 0.95 / 0.85 / 1.14 mm RMSE on a helix 35–50 mm up, 6 ms per iteration, SNR at 40 → 80 mm height 53.8, 39.5, 35.4, 29.2, 28.0, 18.2, 18.1, 16.3, 4.6 dB. Li (arXiv:2201.02372, 2022, *verified*): 16 MLX90393 at 2 mm spacing gave 15 mm average error where 20 at 30 mm gave 3.3 mm, same LM and magnet; a baseline much shorter than the range is ill-conditioned. Cichon, Psiuk, Brauer and Töpfer (IEEE Sensors J. 2019, *abstract*): a cuboid magnet over a CMOS Hall array of up to 36 elements, an analytical (not dipole) magnet model inside a UKF, "sampling rates up to 80 Hz on embedded hardware", 71 µm and 1.4°. The closest published UKF-on-fields product.

**Capsule endoscopy with a particle filter.** Taddese et al. (IJRR 2018, *verified*): six Hall sensors and an IMU in the capsule, an SIR particle filter with random-walk dynamics `x_k = x_{k−1} + v_{k−1}`, `Q = diag(0.0015, 0.0015, 0.0015, 0.01)` "as a trade-off between convergence speed and jitter", 10,000 particles on four i7 cores at 100 Hz, static errors 1.2–5.1 mm and 0.7–3.8° at ~150 mm, converging from random particles with no initial pose.

**Dipole tracking in signal processing.** Wahlström and Gustafsson (IEEE Trans. Signal Process. 62(3), 2014, *verified*): sensor model with the ambient field as a state, `h(x_k) = B_0 + J^m(r_k) m_k`, `x_k = [B_0ᵀ r_kᵀ m_kᵀ]ᵀ`, and analytic Jacobians for any EKF:

```
J^m(r) = (1/‖r‖⁵)(3 r rᵀ − ‖r‖² I₃)                                          (30a)
J^r(r, m) = (3/‖r‖⁵)((r·m) I₃ + r mᵀ + m rᵀ − 5 (r·m)/(r·r) r rᵀ)             (30b)
J^r(r, m) r = −3 J^m(r) m                                                    (31)
```

They prove one 3-axis magnetometer cannot observe scale: scaling r and v by u and m by u³ leaves the output unchanged (their manifold (34)), which is our height/strength trade-off when only one sensor reads the magnet plainly. Their bound (7), `‖J^m(r)⁻¹ J^r(r, Δr)‖₂ ≤ (8/7)‖Δr‖²/‖r‖²`, says when a finite magnet is a dipole. They track with an EKF on a constant-velocity model. Birsan (IEEE Trans. Magn. 2011 and OCEANS 2005, *abstract*) uses a UKF as the proposal of a particle filter over position, moment and velocity because direct inversion of the gradient tensor is "highly sensitive to temporal noise". Ge, Song, Wang and Meng (IEEE Sensors 2021, *abstract*): random-walk system equation with the dipole as measurement model. Hou et al. (ICASSP 2026, title only) is the interacting-multiple-model version.

**AC electromagnetic tracking.** Raab, Blood, Steiner and Jones (IEEE Trans. AES 1979, *abstract*): "linear rotation transformations based upon previous measurements", a Gauss–Newton tracker in 1979 clothes; Kindratenko 2000 and Franz et al. 2014 (records): calibrated EM trackers are limited by static distortion from nearby conductors and steel, mapped over the workspace, not by noise.

**HCI magnet tracking.** uTrack (UIST 2013, *verified*): thumb magnet, two magnetometers, 4.84 mm average 3D error. Finexus (CHI 2016, *verified*): five fingertip electromagnets at 70–125 Hz, four MMC3416 at 320 Hz, FIR band-pass ±2 Hz per tone, Hilbert envelope over 160 samples, trilateration on magnitude; 0.95 mm within 120 mm with fixed orientation, 1.33 mm (σ 1.23) random. AuraRing (IMWUT 2019, *abstract*): 0.1 mm resolution, 4.4 mm dynamic accuracy. MagSurface (arXiv:2105.00543, *verified*): a ~5-step iterative solver seeded from the previous answer, "under 5 ms" on an MCU at 100 Hz, ≤ 2.5 mm. µTouch (arXiv:2601.22864, 2026, *verified*): three MLX90393 at 8 mm; "magnet detected" when any pair of sensors differs by more than 18 µT from a 16-frame background, because an ambient change is common-mode across a small array and a local magnet is not.

**Learning-based.** Sensors 25(20):6444, 2025 (*abstract*): a TinyML regression from a Hall array, 0.026–0.038 mm mean absolute error over a fully sampled 40 × 40 × 15 mm volume at 850–1000 Hz on a plain MCU, i.e. what a lookup-style correction of residual field error can do close in.

### 3.2 The estimators, compared

| Estimator | State | Motion model | Who | Cost | Buys over LM per frame | Fails when |
|---|---|---|---|---|---|---|
| LM / Gauss–Newton per frame, seeded by the last fix | p (3), m linear | none | Schlageter; Hu; Farajidavar; the 2020/2026 Hall papers; us | ~0.3–0.5 ms tracking, ~2–5 ms lattice cold start, 49 ms measured on the seven-seed fallback (§4.4) on the CH32 | exact MAP for the frame, honest per-frame σ | seed far off (Hu 2015: 15 cm); no memory, so weak fixes wander |
| Closed-form init (Hu 2007; Nara) | p, m | none | Hu; Nara; Wu 2026 | µs | seed without search, works at range | noise-amplifying (Birsan); biased by finite differences |
| EKF on fields | [B_0, p, v, m] (12) or [p, v, m] (9) | random walk (Ge), CV (Wahlström) | Wahlström; Ge; Raab in effect | 24 × 9 Jacobian, 24 scalar updates, < 1 ms | the prior enters before fields are compressed to a fix; dropped sensors drop rows; B_0 as a state | linearisation with large P; wrong Q/R; single-sensor scale ambiguity closed only by the prior |
| UKF on fields | same | same | Cichon (36 sensors, 80 Hz embedded, 71 µm); Birsan | 19–25 model evaluations per frame, ~5× EKF | no Jacobian; better at short range where ‖r‖⁻⁵ is far from linear | still Gaussian; cost |
| Particle filter | [p, orientation, …] | random walk | Taddese (10⁴ particles, 4 cores) | beyond an MCU | global convergence, multimodality | impoverishment; jitter unless Q is large |
| IMM | as EKF per model | still + CV + accelerating | Hou 2026 | k × EKF | stop-and-go hands without one compromise Q | tuning |
| Fixed-lag / RTS smoothing | as above | as above | Rauch–Tung–Striebel 1965 *(recalled)* | one backward pass | ~√2 less error on recordings | not causal |
| LM per frame, then Kalman / α-β / 1€ on the output | p, v of the fix | CV | every HCI system above in effect; Casiez 2012 | negligible | jitter/lag trade, gating, coasting, uses the fit's σ | fix stream is not white after in-locator smoothing; cannot recover what the fit discarded |

Three facts tie the table together. An iterated EKF update is a Gauss–Newton step on the frame's residual with the prediction as a prior (Bell and Cathey, IEEE TAC 1993, *recalled*), so "LM per frame" and "EKF on fields" are one algorithm with and without the term ‖x − x⁻‖²_{P⁻¹}. Everything published at 100 Hz on embedded hardware is an EKF/UKF or an iterative solver seeded from the last answer; the particle filter needs a desktop. And no paper reports a large accuracy gain from the filter at good SNR; the gains are at weak signal, through dropouts and in latency, matching our own finding that the fit is noise-limited close in. Motion models in use: random walk (Taddese; Ge) with Q as a jitter/lag compromise; nearly-constant-velocity (Wahlström); Singer's correlated-acceleration model (IEEE Trans. AES 1970, *recalled*), acceleration as a Markov state with τ ≈ 0.1–0.5 s, the textbook hand model.

### 3.3 The hard cases

**Weak signal at range.** Everyone hits the same wall: Farajidavar's 3 cm, the 2026 paper's 60 mm, the 2020 SNR falling 50 dB over 40 mm of height, Finexus beyond 120 mm, our cube-of-height law. Remedies: a prior from the last frames, a closed-form range estimate that degrades gracefully, and a coarser output with an honest bar. With |m| held (our `k`) the single-sensor ambiguity closes: from ‖B‖ = (|m|/r³)√(1 + 3cos²θ), the distance to the strongest sensor is r ∈ [(|m|/‖B‖)^{1/3}, (2|m|/‖B‖)^{1/3}], a 26 % bracket before any fitting.

**Dropouts and glitches.** Finexus and Taddese use windows, which is smoothing. A field-space EKF leaves a missing sensor's rows out of the frame and gates a glitching one per row; that is the one place the full route is structurally cleaner than "fit then filter". Our median-of-3 per axis is the same protection elsewhere, at a frame of delay.

**Array calibration.** Farajidavar's method and `magcal` (alternating pose and sensor fits from a hand-waved magnet, 10.7 % → 2.4 % misfit) are the same method; Iivanainen et al. (2022, *abstract*) reach 1.0 mm, 0.2° and 0.8 % for position, orientation and gain of a fluxgate array from calibration coils. Our remaining 0.05 mT is spatial, not temporal: no time filter touches it, and Farajidavar's 10⁻⁶ mm versus 0.5 mm gap is the same statement.

**Soft and hard iron, ambient field.** Kok and Schön (IEEE Sensors J. 2016, arXiv:1601.05257, *verified*): per sensor `y = D R m + o + e` with `D = C_sc C_no C_si R^bm` lumping scale, non-orthogonality, soft iron and misalignment, and `o = C_sc C_no o_hi + o_zb` lumping hard iron and zero bias, both constant for a rigidly mounted sensor. For a fixed array that is a 3 × 3 and an offset per sensor, which `magcal` can fit (the full 3 × 3 ran away because a magnet at 30–60 mm did not constrain it; one at 15–25 mm would). Schlageter's "1 hour without calibration" and the ±3 µT/°C offset drift say the baseline is a slow random walk: model it so, re-zero when the presence gate has been false for a second, and use µTouch's common-mode test in reverse (a change equal on all sensors is ambient, not the magnet) to decide when re-zeroing is safe.

**Dipole inadequacy close in.** Petruska and Abbott (IEEE Trans. Magn. 2013, *verified*): a nine-term multipole expansion matches FEA within 2 % beyond 1.5 minimum-bounding-sphere radii; the cylinder whose dipole term dominates soonest has diameter-to-length ratio √(4/3). Derby and Olbert (Am. J. Phys. 2010) give the exact cylinder field in elliptic integrals; Cichon used an analytic cuboid a few millimetres from the sensors. Our own two-pole test found L = 0 best, so at ≥ 10 mm with a ~5 × 2.5 mm magnet the dipole is not the problem; the 0.05 mT floor is.

**Ambiguities.** Wahlström's scale manifold (one sensor lit) and the false minimum under a neighbouring sensor at low height (our seed ring): multiple hypotheses (a small particle set or an IMM) are the full cure; a closed-form or lattice seed plus a misfit test is the cheap one.

### 3.4 Sensor spacing against height

Every planar Hall array above with numbers lands in one band: about 1 mm RMS at 3–5 pitches of height with their magnets (10 mm pitch, 20–40 mm → 1.2–2.3 mm; 8 mm pitch, 35–50 mm → ~1 mm; our 20 mm pitch, 16 mm → 0.1–0.25 mm over the footprint, 0.5–0.9 mm over the overhang). Li's 2 mm baseline is the other bound. Per-sensor information about position scales as (∂B/∂r)²/σ² ∝ m²/(σ² r⁸), so σ_pos ∝ r⁴/(m √N_eff), with N_eff the sensors that see the magnet plainly; N_eff grows once the height exceeds the pitch, softening the law toward the cube we measured. Hence: pitch no larger than the lowest working height, so a 2 × 2 block always sees the magnet strongly; beyond ~3 pitches only a stronger magnet, quieter sensors, averaging or a prior helps; and sensors must reach the ends of the tracked area, because outside the footprint all lever arms point one way and scale and height confound. For V6 (shell top 9.4 mm above the elements with the sensors proposed on the MainBoard's underside, tip resting at ~12 mm, §6) a 15–20 mm pitch matches every array above, and the far end of the breadboard (x = 122 … 196 mm on the MainBoard) needs a sensor under it. What the underside costs, by the r⁴ law: against the bench, where 0.16 rows rms was measured with the magnet resting 17.5 mm above the elements, the V6 underside case at ~12.4 mm is (12.4/17.5)⁴ ≈ 0.25 of the bench error bar at the same pitch, and against a top-side placement (9.4 mm) it is up to 3× worse touching and 1.4× at 30 mm of hover, nearer 1.5× once the pitch softening applies; the underside loses nothing that matters provided the pitch is no larger than the rest height.

## 4. Closed-form and far-field estimation

Units follow `MagFit`: mm, mT, moment in mT·mm³ (μ0 m/4π = Br·V/4π: 9,800 for a 6 × 3 mm N52 disc, **4,500 measured for the bench magnet** (`wireless-probe-sensing.md` §6.1, "3,800–5,000 inside the footprint"; a 5 × 2 mm disc computes to 4,530), ≈ 8,700 for the proposed V6 4 × 6 mm N52 rod), `B = 3 r (m·r)/|r|⁵ − m/|r|³` with `r = sensor − magnet`. The numbers "for our geometry" are per raw 100 Hz frame, σ = 0.011 / 0.011 / 0.006 mT on X / Y / Z, the field EMA being off (§5.6); a probe that dwells for four frames gets the tracker's √4, which is roughly the isotropic σ = 0.005 the first draft assumed with a 9,800 magnet, and the scaling that moves every number between the two inputs is σ_pos ∝ σ r⁴/m (equal-error range ∝ (m/σ)^¼: 0.82 from the moment, another 0.82 from the raw X/Y noise) and E ∝ m²/(σ² r⁶) for detection (range ∝ (m/σ)^⅓, 0.77 from the moment). They were computed with `farfield_sim.py`, `crlb.py`, `planefit_gate.py` and `seed_bench.cpp`, re-run with a per-axis σ vector and a moment argument as `gap2_rerun.py` and `seed_bench2.cpp` (the latter compiles against the real `MagFit.cpp`); the re-run with the old inputs reproduces the first draft, so the change is the inputs. The bench packages' rotations (85–92° and 271–272° about z, `MagArrayConfig.h`) are a no-op for the noise model since σ_x = σ_y and sensor Z is board Z.

**The V6 probe magnet (a proposal, not a decision).** §6.1 of the sensing notes settles on one magnet, magnetised along the shaft, at the tip, with the shortest offset the probe allows. Candidates (N52, m = Br V/4π; r_min is the on-axis distance, 2m/r³, at which a sensor directly under the magnet clips at 95 % of range):

| magnet | m (mT·mm³) | r_min at ±40 mT | r_min at ±80 mT | (L/2r)² at r = 12 mm |
|---|---|---|---|---|
| 3 × 6 mm rod | 4,900 | 6.4 mm | 5.0 mm | 0.06 |
| **4 × 6 mm rod** | **8,700** | 7.7 mm | 6.1 mm | 0.06 |
| 5 × 5 mm cylinder | 11,300 | 8.4 mm | 6.7 mm | 0.04 |
| 4 × 10 mm rod | 14,500 | 9.1 mm | 7.3 mm | 0.17 |

With the sensors on the MainBoard's underside (§6) the shell top is 9.4 mm above the elements, so a 4 × 6 rod at a 2 mm offset has its centre ≥ 11 mm up even with the tip fully in a clip, 13 mT on axis, safe at either range; were the parts ever top-side (6.4 mm) the centre would be ≥ 8 mm up, 34 mT, still under ±40. The 4 × 10 rod buys 14 % more range ((14500/8700)^¼) but is no longer a dipole at touchdown; 5 × 5 is the compromise. The tables below use 4,500 (bench, measured) and 8,700 (the proposed rod; 8,600–8,900 over the N52 Br band). State whichever Kevin picks here.

### 4.1 The Nara closed form

**Source.** T. Nara, S. Suzuki, S. Ando, "A closed-form formula for magnetic dipole localization by measurement of its magnetic field and spatial gradients", IEEE Trans. Magn. 42(10):3291–3293, 2006, doi:10.1109/TMAG.2006.879151 (metadata verified via Crossref, PDF paywalled; the equations are quoted from open-access papers that reproduce it). Wang et al. (Micromachines 13:1639, 2022, *verified*) eq. (7)–(8):

```
[ Bxx Bxy Bxz ]   [ x ]        [ Bx ]
[ Bxy Byy Byz ] · [ y ]  = −3 · [ By ]                      (7)
[ Bxz Byz Bzz ]   [ z ]        [ Bz ]

[x y z]ᵀ = −3 · G⁻¹ · [Bx By Bz]ᵀ                            (8)
```

Wu et al. (arXiv:2606.01946, 2026, *verified in full*) make the convention explicit, eq. (6): `A_k(p)(p − p_Ck) = −3 b_k(p)` with `A(p) ≜ ∇b(p)` the gradient tensor at the observation point and `p_Ck` the dipole, and eq. (7) `ρ̂_k = −3 X̂_k^† b̂_k`, reducing to `−3 X̂_k⁻¹ b̂_k` when nonsingular, ρ the *source-to-array* displacement.

**Why.** The dipole field is homogeneous of degree −3 in `r = p − p0`, so Euler's theorem gives `(r·∇)B = −3B`, i.e. `G r = −3 B` with `G_ij = ∂B_i/∂x_j`:

```
r = p − p0 = −3 G⁻¹ B      =>      magnet p0 = sensor p + 3 G⁻¹ B
```

Verified numerically (`check_singular()`): with the analytic `G = (3/R⁵)[(m·r) I + m rᵀ + r mᵀ − 5 (m·r) r rᵀ/R²]` the recovered r matches to 10⁻¹⁴ mm. The moment cancels (B and G are both linear in m): no moment, no iteration, one 3 × 3 solve. Geophysics knows the same relation as Euler deconvolution (Reid et al., Geophysics 1990, *verified*: "the magnetic field of a point dipole falls off as the inverse cube, giving an index of three").

**Conditions.** A single dipole, and B must be the *anomaly* field with the uniform background already removed. G must be invertible: Nara and Ito (J. Appl. Phys. 2014) show it is singular exactly when the moment is perpendicular to r, and that the pseudoinverse still gives the true position there; verified numerically (det G at round-off when m·r = 0, pinv recovers r to 10⁻¹⁴ mm). Higuchi, Nara and Ando (2016) use a truncated SVD, Tang et al. (2023) an eigenvector constraint.

### 4.2 Noise sensitivity

Wu et al. 2026, supplement S.II (*verified*): with `X̃ = X + δX`, `δρ ≈ −X⁻¹(δX) ρ` (S25), `‖δρ‖/‖ρ‖ ≤ ‖X⁻¹‖‖δX‖` (S26); since `‖X‖ = O(‖b‖/‖ρ‖)` for a dipole (S30), the relative position error is `O(‖ρ‖ Δo / (ℓ ‖b‖))` (S31), *range over baseline* times *offset error over field*; with the field term, eq. (12): `δρ ≈ −X⁻¹((δX) ρ + 3 δb)`. Two consequences:

1. Noise σ on a gradient over baseline L: `δr/r ≈ (σ/L)/(3B/r)`, and with `B ≈ 1.4 m/r³`, `δr ≈ 0.24 σ r⁵/(m L)`: error grows as the **fifth power** of range.
2. A uniform field error δB (baseline drift): `δr = −3 G⁻¹ δB ≈ r · (δB/B)`. A 0.05 mT baseline error on a 0.1 mT field misplaces the magnet by half its range. G is immune to a uniform offset; the range is not.

Wu et al.'s measured ladder: raw, affine-calibrated and ideal arrays gave (e_p, e_R) = (107.40 mm, 45.51°), (21.46 mm, 7.51°), (5.66 mm, 0.53°), inter-sensor inconsistency dominating; median solver time 172 µs. The Testolomew log shows the same ordering (0.05 mT repeatable error against 0.005 mT smoothed noise).

### 4.3 The rest of the closed-form family

- **Wynn/Frahm eigenvalue methods** (1972, 1975): the five components of G alone give bearing and moment direction scaled by 1/r⁴, with a fourfold ambiguity that needs a second point (Sensors 24:6194, 2024). Clark 2012 and Beiki et al. 2012 add the Normalised Source Strength `NSS = √(−λ2² − λ1λ3)`, whose isosurfaces are spheres.
- **STAR** (Wiegert and Oeschger, OCEANS 2005/2007, quoted from Wang et al., Sensors 16:2168, 2016, *verified*): `C_T = √(Σ G_ij²) = k (μ0/4π)|M|/r⁴`, `k = 3√(4cos²θ + 2)` from 7.3 at the poles to 4.2 at the equator; range from two points `r1 = Δr[(C_T1/C_T2)^{0.25} − 1]⁻¹`; bearing from ∇C_T with a cube of eight 3-axis sensors; Wang's iteration removes the asphericity in ~4 steps: 3.25 / 2.08 / 2.03 cm RMS at ~1 m with a 300 mm cube. Immune to the uniform field; needs the full tensor at several points, which one planar 4 × 2 cannot supply.
- **Total-field variants** (You et al., Sci. Rep. 2022, *verified*): `|B| = μMs/(4πr³)`, `s = √(1 + 3t²)`, `t = r̂0·m̂0` (eq. 2); treating s as constant, `r = 3|B|/|∇|B||`. Eq. (2) is the source of the `m/r³ ≤ |B| ≤ 2m/r³` bound used below.
- **Two-point and higher-order methods**: Xu (IEEE TIM 2021) subtracts Euler's equation at two points to cancel the background; Wang et al. 2022 use a three-point central difference (RMSE 0.10 / 0.11 / 0.20 m vs Nara's 0.024 / 0.022 / 0.037 m at 62 dB SNR in simulation, better in the field); the finite-difference noise scaling is `n_G = √2 n_B/d_s` (PMC13418459, 2026).
- **Magnetic anomaly detection** (Ginzburg, Frumkis and Kaplan 2002; Sheinker et al. 2009, "detection probability of approximately 0.75 under a false alarm rate of 0.07"; Chenevas-Paule et al. 2025 on a GLRT footing): the array analogue is the presence test in §4.6.
- **Closed form as an LM seed** (Hu, Meng et al., EMBS 2005; PMC4399597): "a coarse estimation … using a linear optimizer, then a refinement … based on the Levenberg-Marquardt nonlinear optimizer using the output of the first step as an initial guess".

### 4.4 Getting G from a planar array, and what it costs

In a source-free region ∇·B = 0 and ∇×B = 0, so G is symmetric and traceless with five independent components; Wu et al. parameterise `X = [[x1,x2,x3],[x2,x4,x5],[x3,x5,−(x1+x4)]]`. For sensors in the plane z = 0:

```
curl-free:        ∂Bz/∂x = ∂Bx/∂z,   ∂Bz/∂y = ∂By/∂z,   ∂Bx/∂y = ∂By/∂x
divergence-free:  ∂Bz/∂z = −(∂Bx/∂x + ∂By/∂y)
```

The five in-plane derivatives are the five unknowns, the `∂Bx/∂y` vs `∂By/∂x` pair is the curl consistency check, and B0 at the centre is the gradient-cancelling weighted mean (the plain average for a centred array). For eight sensors that is a 24-equation, 8-unknown linear least squares, one 8 × 8 solve (`naraSeed()` in `seed_bench.cpp`); rank 5 needs three non-collinear sensors, and the y-derivatives rest on the two-row difference alone.

**Algorithm 1: G and B0 from the eight readings** (~1000 flops):

```c
// unknowns u[8] = { B0x, B0y, B0z, x1, x2, x3, x4, x5 }
c = mean of used s[i]
for each used sensor i:  d = s[i] - c        // dz = 0 for a planar array
    row_x = {1,0,0,  d.x, d.y, d.z,  0,   0  }     // Bx = B0x + x1 dx + x2 dy + x3 dz
    row_y = {0,1,0,  0,   d.x, 0,    d.y, d.z}     // By = B0y + x2 dx + x4 dy + x5 dz
    row_z = {0,0,1, -d.z, 0,   d.x, -d.z, d.y}     // Bz = B0z + x3 dx + x5 dy - (x1+x4) dz
    ata += rowᵀ row; atb += rowᵀ f[i]
solve8(ata, u, atb)      // local 8x8 elimination; do NOT raise MAGFIT_MAX_PARAMS (it sizes the known-strength solver)
```

**Algorithm 2: the estimate with a validity test:**

```c
bool naraSeed(fp, sumSquares, Vec3* p, float* misfit)
    Algorithm 1 -> c, B0, G
    gn = frobenius(G); det = det3(G)
    if (gn == 0 || fabs(det) < 1e-3 * gn*gn*gn) return false   // near-singular: moment ~ perpendicular to r
    r = solve3(G, -3*B0)                                        // r = c - magnet
    *p = c - r
    if (p->z < MAGFIT_Z_MIN || p->z > MAGFIT_Z_MAX || xy outside the fit box) return false
    cost = costAt(fp, *p, sumSquares)   // one variable-projection evaluation, moment solved linearly
    *misfit = sqrt(cost / sumSquares)
    return *misfit < 0.6                // a seed, not a fix
```

Two findings behind it: the plane-fit residual is *not* a usable gate (30–70 % at every height, `planefit_gate.py`), the dipole-explained cost at the estimate is; and skip it when `max|B_i|/min|B_i| > 8` (magnet within ~1–1.5 spacings; the median ratio is 7.4 at z = 30 and 14.7 at z = 20), where the linear model is meaningless and the existing seeds are right.

**Truncation error.** A plane fit over a footprint L at distance R from a 1/r³ source has a relative gradient error of order (L/R)². On the nominal 60 × 20 mm array, noise-free, the fitted G at the centroid is off by 93 % at z = 20, 71 % at 30, 57 % at 40, 36 % at 60, 24 % at 80, 12 % at 120 mm; yet the *Nara position* from the same fit, above the centre, is wrong by only 2.2 mm at z = 20 and 1.5–2.4 mm from 30 to 130 mm, because B0 and G are distorted together and Euler's ratio largely cancels it. Off-centre it does not cancel: 8 mm median at a (30, 20) mm offset for z = 20–40, 17 mm at (60, 40).

**Noise, our numbers** (raw σ = 0.011 / 0.011 / 0.006, 60 trials per pose; the plane fit is unweighted, as the firmware's would be until §9.4 adds per-axis weights). Nara's median error above the centre of the nominal 20 mm grid with the bench magnet, m = 4,500: 2.8 mm at 20, 5.6 at 30, 13 at 40, 30 at 50, 51 at 60, 116 at 80 mm; with the V6 magnet, m = 8,700: 2.0 / 3.0 / 6.8 / 14 / 29 / 72 mm. At a (30, 20) mm offset: 9 / 12 / 20 / 42 mm (4,500) and 9 / 9 / 13 / 23 mm (8,700) for z = 20–50. Rms is 2–5× the median (the G⁻¹ tails). Adding 0.03 mT of per-sensor systematic error: 8 / 20 / 40 mm at 20 / 30 / 40 mm (4,500) and 4 / 9 / 26 mm (8,700); at 0.05 mT, the bench floor: 14 / 31 / 45 and 7 / 19 / 31 mm. On the bench geometry (rows 44 mm apart) with the bench magnet the closed form is never better than ~20 mm (20 / 19 / 21 / 25 mm at 20–50 mm, 38–100 mm with the 0.05 mT floor) and with the V6 magnet 11–16 mm at 30–50 mm: **usable as a direction, not as a seed, until the floor is calibrated down**. On a 20 mm grid it is a seed to ~40 mm with the bench magnet and ~50 mm with the V6 one. (The first draft's "2 mm at 20–30 mm, good to ~35 mm on the bench" assumed m = 9,800 and an isotropic smoothed σ = 0.005.)

**The bound no method beats.** Cramér–Rao for the six-parameter fit, per raw frame, per-axis weighted (JᵀWJ, W = diag(1/σ_axis²)), median over orientations, σ_xy / σ_z in mm above the centre of the nominal array:

| z (mm) | 4,500, free m | 4,500, \|m\| known (σ_z) | 8,700, free m | 8,700, \|m\| known (σ_z) |
|---|---|---|---|---|
| 20 | 0.26 / 0.23 | 0.09 | 0.13 / 0.12 | 0.05 |
| 30 | 1.0 / 0.8 | 0.28 | 0.53 / 0.41 | 0.15 |
| 40 | 2.9 / 2.5 | 0.72 | 1.6 / 1.3 | 0.39 |
| 50 | 8.2 / 6.3 | 1.7 | 4.3 / 3.3 | 0.87 |
| 60 | 20 / 14 | 3.8 | 10 / 7.4 | 1.9 |
| 80 | 70 / 49 | 12 | 37 / 26 | 6.3 |
| 100 | 197 / 128 | 31 | 107 / 69 | 18 |

Fixing |m| barely helps xy and cuts σ_z 2.5–4×. Off-centre the bound is set by distance from the array, not height: at (30, 20) σ_xy is 2.7 / 6.8 / 15 / 32 mm at z = 30 / 40 / 50 / 60 (4,500) and 1.5 / 3.6 / 7.7 / 16 (8,700); at (60, 40) it is already 21 mm at z = 20 and 50 mm at z = 40 with the bench magnet (11 and 26 mm with the V6 one). The bench geometry is 2× worse than the grid below 30 mm (0.52 mm at 20) and, with its 44 mm baseline, slightly *better* beyond 60 mm (14 mm at 60, 51 at 80 for 4,500). Scaling is σ_pos ∝ R⁴/m, so a magnet twice as strong buys 19 % range at equal error. **The "±15 mm" line (`MAGLOC_MAX_ERROR_MM`, free m, per raw frame) is 58 mm above the centre and 50 mm high at (30, 20) with the bench magnet, never at (60, 40); 67 / 59 / 27 mm with the V6 magnet.** For a dwelling probe the tracker's four-frame average moves it to 67 / 60 / 26 mm (4,500) and 77 / 70 / 46 mm (8,700), which is where the first draft's "~80 mm" came from. Nara alone (unweighted) is 4–5× the bound at 30–40 mm and 2.5–4× at 50–60 mm, where the bound itself is 8–20 mm, part of that gap being the weights the bound assumes and the estimator lacks; beyond 60 mm the bound passes 15 mm on every frame. The bench agrees: at 30 mm the raw-frame CRLB on the bench geometry is 1.25 mm xy against the ruler test's 1.7–2.9 mm (x) / 1.9–4.4 mm (y) single-frame scatter with hand jitter and the 0.05 mT floor on top; the first draft's 0.29 mm at 30 mm was never going to be seen.

**Seed benchmark** (`seed_bench2.cpp` against the real `MagFit.cpp`, 120 random poses per height, raw σ, misfit limit 0.4, the real `magFitSolve`, unweighted as today; "ok" = within 10 mm, which separates convergence from noise now that the CRLB itself passes 3 mm at 40 mm; host times, ratios transfer):

| z (mm) | 4,500 nominal: cold ok / ms | seeded ok / ms | seed median err | rejected | 8,700 nominal: cold ok / ms | seeded ok / ms | seed err | rejected |
|---|---|---|---|---|---|---|---|---|
| 15 | 120, 0.028 | 120, 0.008 | 16 | 10 | 120, 0.022 | 120, 0.006 | 17 | 10 |
| 20 | 120, 0.032 | 120, 0.006 | 14 | 3 | 120, 0.022 | 120, 0.005 | 12 | 5 |
| 30 | 113, 0.036 | 113, 0.008 | 12 | 5 | 120, 0.022 | 120, 0.005 | 9 | 4 |
| 40 | 89, 0.066 | 89, 0.021 | 22 | 5 | 115, 0.033 | 115, 0.006 | 11 | 3 |
| 50 | 20, 0.087 | 20, 0.057 | 43 | 12 | 91, 0.064 | 91, 0.009 | 24 | 4 |
| 60 | 1, 0.092 | 1, 0.091 | 84 | 16 | 21, 0.089 | 21, 0.034 | 41 | 9 |
| bench 4,500: 20 | 120, 0.020 | 118, 0.009 | 33 | 23 | | | | |
| bench 4,500: 30 | 119, 0.024 | 119, 0.007 | 27 | 5 | | | | |
| bench 4,500: 40 | 104, 0.064 | 103, 0.013 | 29 | 6 | | | | |

The seeded solve reaches the same answer as the cold start at every height and is 4–5× cheaper at 15–30 mm and 3× at 40 mm with the bench magnet (4–7× to 50 mm with the V6 one); beyond that nothing converges under a 0.4 misfit at raw noise, because the misfit floor of a *perfect* fit (rms noise over rms signal, above the centre) is 0.26 / 0.45 / 0.73 / 1.1 at 40 / 50 / 60 / 70 mm for 4,500 and 0.14 / 0.24 / 0.38 / 0.57 / 0.83 at 40–80 mm for 8,700. On the bench the seed's own error is 27–33 mm at every height: it saves iterations only because the LM basin is wide. On the chip: the host-to-chip factor for the one path measured on both (the tracking fit, 0.0025 ms host against 0.3–0.55 ms chip, inferred from the two-magnet measurement) is 110–220×, so the seeded solve is ~0.5–1.3 ms on the V5F and the lattice cold start ~2–5 ms; the 49 ms in the notes is the seven-seed fallback, not this path (`coldcost_bench.cpp`: below 12 mm `best.z < MAGFIT_COARSE_LOW_MM` sends every cold start through the seven seeds, and at 30–50 mm with 0.03 mT of systematic error the lattice's misfit passes `MAGFIT_COARSE_GOOD_ENOUGH` 20 %, so the seeds run and each `refine` crawls toward `LM_DONE_MM` 0.005 mm on a cost surface whose numeric Jacobian is noise at that scale, until `LM_MAX_ITERATIONS` 30; worse, a *warm* fit that fails `misfitLimit` falls into the full cold path in the same call, `cold = !warm || bestCost > misfitLimit`, so at 40–50 mm a tracking frame costs 77–141 iterations). Even the 110–220× is 2–5× more than the V5F's out-of-order single-precision core should need against a laptop; three unverified candidates are the I-cache state the 49 ms was measured under (`docs/icache.md`: 145× off→on), `refine`'s static arrays in HCLK-waited shared SRAM rather than DTCM, and `sqrtf` going through newlib's errno wrapper (a cross-compile with the board's flags emits 0 `fsqrt.s` and `U sqrtf`; `-fno-math-errno` inlines them; hardware float is otherwise confirmed). Both figures are to be replaced by the `fitUs` histogram (§9.4 step 1).

**The budgeted fit** that follows from this, in `MagLocator::fitFrame`: the invariant is the evaluate budget, not the iteration cap, and a cold start never runs inside a tracking frame:

```c
budget = 60 evaluate calls                         // ≈ 1 ms at the inferred chip factor
if (track predicts) start = x̂⁻  else if (coldPending) start = seed[coldIndex]
cost = refine(start, maxIter 12, doneMm 0.02, relCost 1e-3, budget)
if (misfit > limit && warm) { fix.valid = false; coldPending = true; return; }   // no cold start in this frame
if (coldPending) { one lattice z-slab (36 costAt) or one seed per frame; keep best; coldIndex++; }
if (misfit > loo_threshold && budget left) refine once without the worst-residual sensor
```

`LM_MAX_ITERATIONS` 30 → 12 and `LM_DONE_MM` 0.005 → 0.02 (the CRLB at 20 mm is 0.13–0.26 mm; a 5 µm stop is far below it), plus a relative-cost stop; the seeds one per frame with a best-so-far in locator state; the lattice sliced by z-plane if it stays. Capping alone cuts the 40–50 mm warm-fit cost by ~40 % (0.100 → 0.062 ms host at z = 50) and is not sufficient without the budget.

**The verdict on this array.** `MagFar` was built from exactly this, measured on the bench geometry, and shelved (`attic/README.md`): the sign convention checks out (a magnet 150 mm up is found within 4 mm from perfect readings), but "on this array the gradient is a finite difference across 54 × 44 mm, and at every distance where there is signal that is not a derivative": 25–50 % of the distance in error at 50–70 mm with no noise, 10 % only beyond 90 mm where the magnet reads 0.006 mT. The lattice search `magFitCoarse` replaced it: no iteration either, and within one step of the answer at any distance. Both pictures are consistent with the numbers above (the closed form works above the centre of a 20 mm grid and badly off-centre or on a 44 mm baseline). Keep the formula and the algorithms on file for a V6 array at ≤ 20 mm pitch with the per-sensor affine calibration done; on the bench the lattice search is the seed.

### 4.5 Cruder fall-backs, with their errors

Computed with `crude()` on the nominal array, 200 orientations per pose:

- **Range from the strongest sensor's |B|, moment known.** From |B| = m s/r³, s ∈ [1, 2]: r ∈ [(m/|B|)^{1/3}, (2m/|B|)^{1/3}], a spread of 2^{1/3} = 1.26. The geometric mean r̂ = (√2 m/|B|)^{1/3} is within ±12 % always; its rms error (raw frames, above the centre) is 1.8 mm at r = 24, 3.2 at 42, 9.3 at 62, 24 at 81, 41 at 101 mm with the bench magnet, and 1.8 / 2.8 / 5.4 / 15 / 28 mm with the V6 one: the ±12 % orientation term dominates to ~45 mm, the noise term σ/(3|B|) beyond, since the strongest sensor reads only 0.013 mT (4,500) or 0.023 mT (8,700) at 80 mm against σ = 0.011. Off the footprint at (60, 40) multiply by 3–5. The one crude estimate worth having; it needs the moment, which a probe has. The ±12 % is an orientation ambiguity, not noise, and averaging does not shrink it.
- **Direction from the |B|²-weighted centroid** (what `magFitSolve` seeds with): 3.6–4.4 mm xy above the centre at every height to 100 mm, but bounded by the footprint, always biased *toward the array centre* and growing with height (12 mm at a (30, 20) offset and z = 20, 50–70 mm at (60, 40)). The strongest-sensor heuristic is never better.
- **Ratio of two sensors' |B|** for a moment-free range: useless here, 30–400 mm rms, because the orientation factor differs between sensors by up to 2^{1/3}; the gradient tensor is the fix.
- **Which side of the array.** With 3-axis sensors in a plane there is no mirror ambiguity, and the probe is always above the board: a closed-form estimate with z < 0 is a rejection, not a mirror.

**Algorithm 3: crude range and direction, with its bar** (when the seeded fit rejects: 60–80 mm with the bench magnet, 70–100 mm with the V6 one, +15–25 mm with the 4-frame block of §4.6):

```c
E = sum over used raw-frame axes of (f/sigma_axis)^2  // sigma per axis from the live baseline residual, raw frames
present = (E > 43) in 3 consecutive raw frames          // ~1e-6 per triple; clear only when E < 35
k = argmax |f[i]|; Bk = |f[k]|
rangeGeo = cbrt(1.4142 * m / Bk)                        // m = known strength
sigmaRange = rangeGeo * sqrt(0.12^2 + (sigma/(3*Bk))^2) // ±12 % orientation term plus noise
dir: from the Nara/gradient direction while Nara is within ~30 mm (to 50 mm with the bench magnet, 60 mm with the V6 one;
     G ignores uniform drift) else the |f|^2 centroid
report sigma_xy = 15 mm (gradient direction) or 25 mm (centroid), sigma_z = sigmaRange
```

Where each estimate is worth using on the nominal 20 mm array, raw per-frame σ, for the bench magnet (4,500) and the proposed V6 rod (8,700); halve the distances on the bench geometry until it is re-calibrated:

| use | 4,500 (bench magnet) | 8,700 (V6 rod) | expected error |
|---|---|---|---|
| existing seeds + LM (peak/min ‖B‖ > 8) | < 30 mm | < 30 mm | CRLB 0.1–1.0 mm |
| seed → LM refine every frame, over the footprint | 30–45 mm | 30–55 mm | seed 6–13 mm (4,500) / 3–9 (8,700); LM at the CRLB, 1–5 mm |
| seed → LM, accept on misfit, just outside the footprint (the (30, 20) pose) | 30–40 mm | 30–50 mm | seed 9–20 mm (4,500) / 9–13 (8,700); LM 2–7 mm |
| seed → LM as a *rough* fix, bar ~20 mm, misfit ≤ 0.7, report the fit's own σ | 45–60 mm | 55–70 mm | CRLB 5–20 mm (4,500), 6–21 (8,700); the misfit gate binds at 59 mm for 4,500, the 20 mm bar at ~70 mm for 8,700 |
| Algorithm 3: presence, direction, ‖B‖ range | 60–80 mm (95 with a 4-frame block) | 70–100 mm (125 with the block) | "over the array, this high, ±25 mm" |
| absent | > 80 mm (> 95) | > 100 mm (> 125) | |

With the bench magnet on raw frames the *fix* range is ~60 mm and the *presence* range ~80 mm, so the LED and LCD "far" state starts at about a hand resting beside the board, not 100 mm; the 4 × 6 mm rod buys the 100 mm back; a further doubling of the moment is worth only 19 % more; and the per-sensor floor (0.05 → 0.01 mT) is worth more than any magnet the probe can carry.

### 4.6 Presence detection at range, and the baseline

**Statistic.** With ambient subtracted and per-axis noise σ, `T = Σ (b/σ)²` over the 24 readings is χ²(24) under "no magnet" (mean 24, sd 6.9); a magnet adds `E = Σ|B_i|²/σ²`. Thresholds (Wilson–Hilferty, checked by a 4000-trial Monte Carlo): T > 43 for Pfa = 10⁻² per frame, > 51 for 10⁻³, > 59 for 10⁻⁴, > 73 for 10⁻⁶. **Run it on the raw per-frame averages (σ ≈ 0.011 mT), not on the smoothed fields**: after the median-of-3 and the weak-signal EMA (α = 0.08, ~12-frame memory) consecutive frames are strongly correlated and a three-in-a-row rule buys nothing there. On raw frames, T > 43 in three consecutive frames (~30 ms at the real frame rate) is ~10⁻⁶ per triple for two frames of latency. Use the per-axis σ, `T = Σ (b/σ_axis)²`, still χ²(24) under the null, so the 43 / 51 / 59 / 73 thresholds do not move. The **bench magnet (4,500)** above the centre gives E ≈ 76 at 60 mm, 18 at 80, 5 at 100 (worst orientation 30 / 5 / 1.4): single-frame detection at Pd ≈ 0.5 reaches **79 mm** at the 10⁻² threshold (T > 43) and 71 mm at the 10⁻⁴ one (T > 59, E = 35), and a 4-frame block average **97 mm** (89 at 10⁻⁴), where the strongest sensor reads 0.0065 mT. The three-in-a-row rule at 43 needs Pd_single ≈ 0.8 (E ≈ 27) and lands a few mm inside the single-frame figure. The **V6 magnet (8,700)** gives E ≈ 300 / 57 / 18 at 60 / 80 / 100 mm: **98 mm** single frame (90 at 10⁻⁴), **124 mm** with the block (115). The bench geometry is 3 mm shorter in each case. (The first draft's "E ≈ 160 / 35 / 10, ~85 mm single frame, ~100 mm with the block" was the 9,800 magnet.) Against `MAGLOC_PRESENT_MT = 0.06 mT` on one smoothed axis (12σ) the pooled statistic still gains 20–25 % in range; the real gain is a calibrated false-alarm rate. The stronger, costlier detector is the dipole-explained energy `sumSquares − cost` from `costAt()` (the GLRT with the dipole as basis).

**Drift at that level.** The signal at 100 mm is 0.0065 mT (4,500) or 0.012 mT (8,700), below Earth's 0.05 mT by an order of magnitude and below 5 °C of die warming (0.015–0.05 mT per sensor, not uniform across sensors); the bench log records 0.1 mT in ten minutes. Rules: (1) the baseline is a slow per-axis state (α_baseline = 1 − exp(−dt/τ), τ ≈ 30 s, i.e. ~3 × 10⁻⁴ at a 10 ms frame, never a per-frame constant since the frame is 10–14.5 ms today) updated only while E < 35 and frozen above 43; (2) never re-zero a stuck-high T blindly: first ask whether one dipole at a plausible position explains the excess (the existing `MAGLOC_BASELINE_MAGNET_*` check does this on a whole baseline; run it before any re-zero after > 60 s of high T); (3) prefer the gradient where possible, since G ignores any uniform offset; per-sensor drift only calibration removes; (4) measure σ from the live baseline residual, not the datasheet, and store it with the baseline. Keep `z` on the console as the manual override.

**Cost.** Algorithms 1–2: 24 row accumulations, an 8 × 8 and a 3 × 3 solve, one `costAt` (8 kernel evaluations), well under 50 µs on the CH32H417; Algorithm 3 negligible; the LM refine from a seed is the 2–6 iterations it already does when tracking, ~0.3–0.5 ms on the V5F (§4.4).

## 5. Motion filtering and inference

### 5.1 What a hand can do

**Speed.** Casiez, Vogel, Balakrishnan and Cockburn (HCI 23(3), 2008, *verified*): "The maximum logged limb speed was 2441 mm/s, with a 97th percentile value of 1536 mm/sec" for a mouse on a desk; "participants were limited by a maximum limb speed of about 1.5 m/sec"; mean peak velocity per trial ~280–300 mm/s. At 100 Hz that is 15 mm per frame at the 97th percentile, 24 mm at the extreme, ~3 mm for a typical ballistic peak. A breadboard is a smaller workspace, so 1.5 m/s is a generous bound. Vogel and Balakrishnan (UIST 2005, *verified*) use a pause threshold of 80 mm/s and a low-pass whose cutoff interpolates between 0.25 Hz below 10 mm/s and 5 Hz above 200 mm/s, the direct ancestor of the 1€ filter.

**Acceleration.** From the minimum-jerk model (Flash and Hogan 1985, *recalled*, constants checked numerically) x(τ) = A(10τ³ − 15τ⁴ + 6τ⁵), v_peak = 1.875 A/T and a_peak = 5.77 A/T²; a 100 mm reach in 0.4 s gives 470 mm/s and 3.6 m/s²; a 20 mm hop in 0.2 s gives 190 mm/s and 2.9 m/s²; a 50 mm hop in 0.15 s is 12.8 m/s² and a 200 mm sweep in 0.25 s 18.5 m/s². Fast reaches really do exceed 10 m/s²: Gaveau and Papaxanthis (PLoS ONE 2011, *verified*), fingertip tracked at 120 Hz over 45° shoulder rotations, used onset thresholds of "1 m/s2, 3 m/s2 and 10 m/s2 for slow, natural and fast speeds respectively" with durations 0.84 / 0.64 / 0.42 s, and minimum jerk puts their 0.55 m fast swing at 18 m/s². So a_max ≈ 10–20 m/s² is a hand, though a breadboard's shorter reaches mostly stay under 5. A resting hand is very quiet: Duval and Jones (Exp. Brain Res. 2005, *verified* abstract), unloaded index-finger tremor of 20 subjects, "0.0973 mm in displacement units, 4.525 mm s(-1) in velocity units, and 301.526 mm s(-2) in acceleration units", 60 % of the acceleration in the 16.5–30 Hz band; that verifies the dwelling σ_a ≈ 0.3 m/s² and puts the resting per-frame second difference at a·T² ≈ 0.03 mm. Use ~0.3 m/s² as the hand's dwelling acceleration and 2–3 m/s² as a typical reach, and note that the filter's Q is a different question (§5.2–5.3).

**Tremor.** Peaks at 2–4 Hz and 8–12 Hz, the latter growing with accuracy demand (Morrison and Newell 2001, *recalled*). Amplitude (*verified*): Stiles 1976, normal hand tremor "rms displacement amplitude … control levels of about 30 µm" at "8–9 Hz"; Ang 2004 §1.1, "an oscillation at 8–12 Hz … as large as 50 µm peak-to-peak in each of the principal axes [Hunter 93] … 38 µm rms [Singh 02]", and "jerk … and drift … are often larger than physiological tremor [Riviere 97]". So tremor is tens of micrometres, not the 0.5–0.9 mm sometimes assumed for it; what the bench's "0.9 mm rms hand scatter" is, is drift (§5.2). Pavlovych and Stuerzlinger (EICS 2009, quoted in Casiez 2012, *verified*): "Jitter should be less than 1 mm mean-to-peak, but lag should be below 60 ms", and with 40–50 ms of system lag "that leaves less than 10–20 ms for the filter". Wacom pads sample at 200 pps and iPads at 240 Hz with a Pencil near; our 100 Hz is at the low end, which argues for small filter lag rather than more smoothing.

**So a glitch is:** between consecutive frames (per 10 ms; scale by dt, since the frame is 10–14.5 ms today), < 3 mm is ordinary, 3–15 mm a fast real move, 15–25 mm possible only at the top of human speed, and anything beyond 25 mm (ten rows, 2500 mm/s) in one frame is not the hand. Acceleration cannot be gated the same way: a hand's peak of 10–20 m/s² moves the per-frame second difference by only a·T² = 1–2 mm, while the noise-driven second difference of fixes with 0.9 mm scatter is √6 · 0.9 ≈ 2.2 mm rms (variance 6σ² from the (1, −2, 1) stencil), 1.2 mm at the 0.5 mm gating floor. A 10 mm reversal between frames is 200 m/s² (v from +1000 to −1000 mm/s in 10 ms), twenty times any hand, but it is also 6–11 √S to the χ² gate of §5.2 and is rejected there; a threshold set at a real hand's 10 m/s² (1 mm per frame²) would reject two thirds of the frames of a still hand at σ = 0.9 mm and 41 % at 0.5 mm. **So there is no acceleration or reversal rule**; unrealistic acceleration is caught by the innovation gate with an honest R, and any second-difference threshold added later is a noise statistic, set from the 99th percentile of |x_k − 2x_{k−1} + x_{k−2}| over 30 s of wiring (≈ 2.6 · √6 · σ, i.e. 3–6 mm, 30–60 m/s²) and subordinate to the gate. The useful thing the second difference does is the opposite: at rest it measures the fix noise alone, since the hand contributes 0.03 mm per frame² and a 0.5 Hz, 2 mm sway 0.002 mm, so σ̂_R = std(Δ²x)/√6 (the pseudo-residual estimator of Gasser, Sroka and Jennen-Steinmetz, Biometrika 1986, *recalled*) is the honest R floor, expected 0.4–0.9 mm. (The first draft's "a reversal of > 10 mm is an acceleration above ~10 m/s²" was off by twenty.)

### 5.2 Gating, the floor, and reinitialisation

**Innovation (chi-square) gating.** With prediction x̂⁻, covariance P⁻, measurement z, H and R: ν = z − Hx̂⁻, S = HP⁻Hᵀ + R, d² = νᵀS⁻¹ν, and the gate is `{d² ≤ τ}` with τ a chi-square quantile so that P{accept} = P_g under the nominal model (Or, arXiv:2512.18508, 2026, *verified*; Bar-Shalom, Li and Kirubarajan 2001, *recalled*). Quantiles (computed, checked against the paper's 5.991): 3 DOF 6.25 (90 %), 7.815 (95 %), 11.345 (99 %), 16.27 (99.9 %); 2 DOF 4.605, 5.991, 9.21, 13.82; 1 DOF 2.706, 3.841, 6.635, 10.83. Or's paper also shows that gating *contracts* the post-gate innovation statistics, so R must not be re-estimated from gated innovations.

**The floor.** If S is too small every ordinary at-rest fix is rejected and the track freezes. Two remedies, both standard in radar tracking: a floor under R and real process noise in Q, and track deletion or reinitialisation "if the target was not seen for the past M consecutive update opportunities (typically M = 3 or so)" or when the covariance passes a threshold; tracks are confirmed by an M-of-N rule with M = 3, N = 5 typical (Wikipedia "Radar tracker", *verified*). The floor here is 0.5 mm per axis, and it is *not* the hand. The first draft read the ruler test's "0.9 mm rms in xy, 0.5 in z" (`wireless-probe-sensing.md`) as hand jitter; re-fitting that recording's raw `mag` stream frame by frame with no smoothing (`holdstill.cpp`, 16 stationary windows) gives per-axis rms about the mean 2.37 / 2.86 / 1.93 mm, the fit's own σ 2.31 / 2.91 / 1.85 and a white part (consecutive differences/√2) of 2.18 / 2.69 / 1.86 mm: at 30 mm with the weak magnet the single-frame scatter is all fit noise, and the slow remainder √(2.37² − 2.18²) ≈ 0.93 / 0.97 / 0.5 mm is exactly the smoothed figure, drift over seconds, which is why the 1€ only reached 0.55 mm. The all-orientations recording (strong magnet at 11–13 mm, fit σ 0.05–0.11 mm, so the scatter *is* the hand) shows rms 0.3–1.3 mm per axis whose structure function rises steadily from 0.28–0.42 mm at τ = 85 ms to 0.9–1.6 mm at 0.7 s: smooth drift at 3–7 mm/s, not an oscillation, and a 5.7 s resting window under 0.16 mm at every lag. The constant-velocity model tracks drift, and it leaks into the innovation at only 3–8 % (|1 − H(e^{jωT})| of the one-step predictor at ≤ 1 Hz, `gap1_cv_analysis.py`); tremor is tens of micrometres (§5.1). So at rest the innovation is σ_fit ⊕ tremor ≈ 0.1–0.3 mm per axis against √S = 0.6–0.8 mm, and the 99 % gate rejects ≈ 0 % (60 s simulations at 100 Hz, `gap1_rest_sim.py`: 0.00 % up to 0.3 mm of tremor, ≤ 0.5 % at an unphysical 0.5 mm, for every candidate Q; even taking 0.9 / 0.9 / 0.5 mm *as* innovation the tail is 10 %, not 30 %). The floor's job is the repeatable 0.05 mT array error the fit's bar under-reports by up to half; a noisy fit carries its own σ. Measure it as σ̂_R from the second difference at rest (§5.1) and raise it only if the mean d² at rest exceeds 3. **Keep this floor separate from the LED drawing floor (0.8 mm, §7)**: one is for accepting fixes, the other for not drawing a patch narrower than a third of a pitch.

**The gate must leave room for the hand's acceleration, and must not starve the velocity.** Because a_max T² (1–2 mm) is larger than the honest R (0.5 mm), the innovation of a real reach carries a bias that a tight gate rejects, and a rejected frame never updates v̂. Two independent simulations of the first draft's proposal (Q switched between 300 and 2500 mm/s² on the *filtered* speed, hard rejection at d² ≤ 11.3) found the same deadlock: minimum-jerk hops and reaches through the §9.2 filter reject 19 of 20 frames of a 20 mm hop in 0.2 s and 37–53 of 40–60 frames of a 100 mm reach in 0.4 s (`gap1_move_sim.py`, `gap7_minjerk.py`; the gate alone, reinit not simulated). At σ_a = 300 the filter's bandwidth is 2.8 Hz, an ordinary 3 m/s² onset crosses the 1.5–2 mm gate by frame 4–5, v̂ stays at zero, the filtered speed never reaches the 20 mm/s switch, and the dwelling Q stays in force until the reinit rule re-seeds the track with v = 0 and P_vv = 500² (`startTrack`), after which the wide S accepts a few frames, P shrinks, the gate closes and the cycle repeats: the track follows in steps of a few frames with the 1€ reset each time, a staircase, not a track. Bar-Shalom's rule for the white-noise-acceleration model, σ_a between 0.5 a_max and a_max (*recalled*, §6.2.2), and Kalata's tracking index say the same from the other side. What the two simulations agree on: the speed-switched 300/2500 is dead; a **Huber soft gate** (down-weight before dropping, §5.3) is mandatory so v̂ keeps moving through a fast onset; the R floor stays at 0.5 mm; and the human-speed bound is the guard when the gate cannot be. Where they differ, on σ_a, §5.3 gives both settings.

**Reinitialisation.** N consecutive rejected fixes that agree with *each other* mean the track is wrong, not the fixes. The agreement test must use the human speed bound (2500 mm/s × dt, 25 mm at 10 ms), not S: two true fixes from a hand at 500 mm/s are 5 mm apart, which S (~0.7 mm) would call inconsistent, and the track could then only ever recover while the hand is still. The covariance-independent check, a jump over 2500 mm/s × dt is a glitch whatever P says, applies **unconditionally**, not only while the track is under 3 frames old: a young track's S (P_vv at 500 mm/s, S = 26 mm², 99 % gate at 13 mm) and the coast covariance after a dropout (§5.4) both open the χ² gate so wide that the speed bound is the only guard there.

### 5.3 Other robust recipes

**Hampel / median-of-N.** For 2k + 1 samples, x̃ = median, MAD = median|xᵢ − x̃|, σ̂ = 1.4826 MAD, outlier when |x − x̃| > 3σ̂. The median-of-3 on the sensor axes is the k = 1 case: one frame of delay, kills single-frame spikes, not two-frame ones.

**α–β and α–β–γ.** Per axis: x̂ ← x̂ + T v̂; r = z − x̂; x̂ ← x̂ + α r, v̂ ← v̂ + (β/T) r; Kalata's tracking index Λ = σ_w T²/σ_v gives the steady-state Kalman gains, r = (4 + Λ − √(8Λ + Λ²))/4 *(recalled)*, α = 1 − r², β = 2(2 − α) − 4√(1 − α) *(verified)*. The cheapest thing that carries a velocity.

**Motion models.** Constant velocity with white-noise acceleration, per axis (Bar-Shalom, *recalled*):

```
F = [[1, T], [0, 1]]
Q = σa² · [[T⁴/4, T³/2], [T³/2, T²]]          // σa in mm/s², T = dt, the measured interval between frame stamps
// σa = 2000 mm/s² gives Q ≈ [[0.01, 2], [2, 400]] at dt = 0.01 s and [[0.024, 3.9], [3.9, 625]] at 0.0125 s
```

T is never a constant: the frame is 10–14.5 ms today (§2.4), and a fixed T under-states the position process noise by (dt/T)⁴. `MagTracker.cpp` already does this right (it computes Q from the dt it is handed in `predictAxis`, the 1€ α from `oneEuroAlpha(cutoffHz, dtS)`, the coast timers and decay from dtS, and `MagLocator.cpp:190` measures dtS from `micros()`), which is what Casiez, Roussel and Vogel prescribe ("we rewrite equation 1 to take into account the actual time interval between samples"); it is the locator's per-frame IIR constants and this document's "per frame" figures that had to be corrected (§2.10 item 11).

Singer's model makes acceleration an exponentially correlated process with τ_m ≈ 0.1–0.5 s (a hand's accelerations last 100–300 ms); IMM (Blom and Bar-Shalom 1988, *recalled*) runs a still and a moving model and mixes them by likelihood. For one target on a microcontroller a two-level process noise, "the poor man's IMM", gets most of the benefit, on two conditions that the first draft's 300/2500-on-filtered-speed missed (§5.2): the dwelling level must not gate out an ordinary onset, and the switch must not depend on fixes the gate has dropped. Two settings survive the simulations, each tested where the other was not:

- **(A) two-level, 1000 dwelling / 2500 moving, Huber to 8× the gate, a significance-tested switch** (`gap1_move_sim.py`, `gap1_rest_sim.py`). At σ_a = 1000 the bandwidth is 5 Hz and a 20 mm hop is followed with 0 of 20 frames rejected and 1.0 mm of peak error (19 of 20 at 300); with the Huber weight w = min(1, τ/d²) applied as R/w for τ < d² ≤ 8τ (d² ≤ 90.8, a single-axis |ν| up to 9.5 √S ≈ 6.6 mm) and a drop beyond, a 100 mm reach at 470 mm/s passes with 0 of 40 dropped, the switch fires at +110 ms and the peak error is 1.3 mm (36 of 40 dropped by a hard gate). The switch is `moving = |v̂|₀.₃ₛ > 50 mm/s && |v̂|² > 9 ΣP₁₁`, back to dwelling below 30 mm/s; the significance term matters because v̂ from fit noise alone is 37–40 mm/s rms with a 2.5 mm fit, and without it a still hand reads "moving" 36–77 % of the time. √S is 0.69 dwelling / 0.82 moving; a 5 mm glitch (d² = 53, w = 0.21) moves the track 0.8 mm, a 10 mm one is dropped. Tested at rest with clean and noisy fits and on reaches to 3.6 m/s²; not tested against 10–18 m/s² sweeps.
- **(B) single σ_a = 10000, 5σ gate, Huber from 3.4σ** (`gap7_minjerk.py`). With d² ≤ 25 per axis for rejection and w = min(1, 3.37/√d²) between, every reach to 18.5 m/s² (200 mm in 0.25 s) passes with no rejections and under 2 mm of transient error; false rejection at rest is 0 in 20 000 frames. The cost: √S = 1.31 mm, K₀ = 0.85, so the Kalman is nearly pass-through and a single-frame glitch of 2–6 mm enters the track, which the 1€ at 1 Hz turns into 0.3 mm of cursor motion at rest but shows during a move; glitch rejection in that band moves to the measurement-space tests below. Tested on reaches to 18.5 m/s²; not tested at rest with a noisy fit, and its coast covariance grows too fast to draw (§5.4).

**Default for the bench: (A)**, because it was tested at both ends of a breadboard's workspace and (B)'s hard cases are 200–300 mm sweeps beyond it; the bench test that decides is the rejection count during a deliberately fast sweep (§9.4): if (A) drops frames there, σ_a,moving rises toward 10000, and if a two-level Q is kept for its coast behaviour it may also be switched on the innovation rather than the speed, with P_vv re-inflated to (300 mm/s)² on the switch. Either way: Huber, no reversal rule, unconditional speed bound. The tree's single 3000 is nearer either than the first draft's 300 was. Note that neither Q makes the Kalman a rest-time smoother: the 10 Hz leak into the innovation is still ~1 at σ_a = 2500 and only drops below 0.3 at ~40 000 (α = 0.97, no filter), so a larger σ_a lowers rejections only by inflating S, which the floor does directly; the Kalman's job is gating, velocity and coasting, and the 1€ downstream does the perceptual smoothing (§5.5).

**Huber / robust Kalman**: weight the measurement by w = min(1, c/√d²) (or τ/d² in the quadratic form) with c around the 95 % gate radius, scale R by 1/w, the one-step form of the IRLS robust filters (Gandhi and Mili 2010, *recalled*). A soft gate: a mildly surprising fix is down-weighted, not dropped, and, as the simulations above show, that is what keeps v̂ alive through a fast onset.

**RANSAC over sensors.** A leave-one-out refit is one warm `refine`, ~0.3–0.5 ms on the V5F (§4.4), so run at most one per frame, on the sensor with the largest |residual|/σ, and only when the misfit exceeds the threshold, never eight in a frame (the first draft's "10–25 ms" was the seven-seed cold-path cost, not a refit's); a sensor whose removal lowers the misfit by a large factor is the glitching one. This is the measurement-space test, the gate the state-space test, and they catch different failures (a corrupt I²C read versus a fit that converged to a wrong but self-consistent answer). Under setting (B) it also carries the 2–6 mm glitch band the wide gate lets through.

### 5.4 Coasting

Radar practice: propagate the track by the model when no plot associates, let P grow, drop after M misses or a covariance threshold. Two quantities size the horizon, and they must not be conflated: the filter's own stochastic σ, √P₀₀ ≈ σ_a √(T t³/3), which is also the radius the next fix is gated against, and the deterministic bound of a hand that starts moving during the dropout at a constant unknown acceleration, a t²/2, with *the hand's* a (0.3 m/s² dwelling from Duval and Jones, ~2.5 m/s² for a typical reach), which is not the filter's σ_a (the first draft put σ_a,dwell into the deterministic formula; at 1000 mm/s² that is 45 mm at 0.3 s, half a V5 board, and meaningless). Numbers: under setting (A) 2√P₀₀ is 3.7 mm at 0.1 s and 8.8 mm at 0.18 s dwelling, 9 mm at 0.1 s moving; the hand's deterministic bound is 1.5 / 4.9 / 13.5 mm at 0.1 / 0.18 / 0.3 s dwelling and 12.5 mm at 0.1 s moving. Draw the larger of the two as the ring (under (A) the filter's own belief dominates while dwelling, and a hop that starts in the gap is caught by the gate and the speed-bound reinit, not by the ring); under setting (B) P grows to 18 mm at 0.1 s and 95 mm at 0.3 s, too fast to draw, so there the hand's bound is the ring and, after any dropout, the χ² gate is wide open and the unconditional speed bound is the only guard. Coast the *track* for up to 300 ms (at 0.3 s dwelling under (A) √S ≈ 9.6 mm, so a fix within ≈ 32 mm re-associates, which covers a hop that started in the gap), but stop drawing the cursor once the ring passes 5 mm: ≈ 130 ms dwelling and ≈ 65 ms moving under (A). Damp v ← v · exp(−dt/τ) with τ ≈ 0.15 s (≈ 0.94 per 10 ms frame, not the 0.9 the first draft wrote, which is τ ≈ 0.095 s) so a flick does not carry the cursor off the board; the existing `MAGTRACK_COAST_TAU_S 0.15` does exactly this. While coasting the row counter must not change its counted row; while lost the counted row is kept for the tap detector's 0.3 s tolerance and then cleared. Feed the prediction x̂⁻ to the LM fit as its start on every frame, including after a failed fit: fewer iterations after a gap, and no seven-seed cold start.

### 5.5 The 1€ filter

Casiez, Roussel and Vogel (CHI 2012, *verified*):

```
X̂ᵢ = α Xᵢ + (1 − α) X̂ᵢ₋₁            (1)
α = 1 / (1 + τ/Tₑ),   τ = 1/(2π f_c)   (4), (5)
f_c = f_cmin + β |Ẋ̂ᵢ|                  (7)
```

The speed is computed from the raw signal and low-passed at a fixed 1 Hz. Tuning, verbatim: "First β is set to 0 and f_cmin to a reasonable middle-ground value such as 1 Hz. Then the body part is held steady or moved at a very low speed while f_cmin is adjusted to remove jitter and preserve an acceptable lag during these slow movements. Next, the body part is moved quickly in different directions while β is increased with a focus on minimizing lag. … Rotational input uses a similar tuning process, but rotation axis and angle are filtered separately." Their desktop values were f_cmin = 1 Hz, β = 0.007 with speed in px/s at 109 PPI (≈ 0.03 per mm/s); in their comparison 1€ matched a 14-sample moving average's jitter with "almost imperceivable" lag, and the Kalman filter's lag "was comparable to the moving average". The intuition: "people are very sensitive to jitter and not latency when moving slowly, but as movement speed increases, people become very sensitive to latency and not jitter." Pitch Pipe (Taranta et al., GI 2019, *verified*) auto-tunes it from a PSD noise estimate and a maximum-speed task to f_cmin 0.1–0.5 Hz and β 0.005–0.03. N-euro Predictor (Shao et al., IMWUT 2023, *verified*): in a user study 1€ was rated equal to or better than the neural predictor on jitter while predictors won on lag, and overall ranking followed jitter.

For us at 100 Hz in millimetres: f_cmin 1.0 Hz, β 0.03–0.045 per mm/s (f_c ≈ 4 Hz at 100 mm/s, ~10 Hz at a 200 mm/s sweep), d_cutoff 1 Hz; α(1 Hz) = 0.059, α(4 Hz) = 0.20. The tree has β = 0.1 (300 mm/s → 31 Hz, 1.5 mm of lag); the paper's procedure decides between them on the bench. The shaft direction gets its own filter with f_cmin 0.5 Hz and β 0.005, using the *tip speed* as the derivative input so a still, tilting hand is smoothed hard; filter (dx, dy) of the unit vector and renormalise.

### 5.6 The double-smoothing caveat

The locator smooths fields for ~10 frames at weak signal (`MAGLOC_SLOWEST_ALPHA 0.08`) and medians each axis over 3, so fixes are correlated over N_s ≈ 3–10 frames, and a filter that takes them as independent over-trusts them by about N_s. Options, cheapest first: (1) inflate, R_eff = N_s (σ_fit² + σ_sys²), N_s read from the locator's current smoothing (`MagTrackInput.alpha` carries it), right in steady state and wrong only in the transient; (2) update every N_s-th frame with un-inflated R and predict between; (3) switch the field smoothing off when the track is on and let the Kalman smooth. **The decision is (3)**: information is used once, σ_fit is white again, the presence test (§4.6) wants raw frames anyway, and the median goes with the SET_COUNT check (§2.10). (1) is the interim while (3) is being tested, and Bryson and Henrikson's state augmentation for correlated measurement noise (AIAA 1967, *recalled*) is the textbook fourth way and not worth it here.

### 5.7 The full route: an EKF on the fields

State x = [p (3), v (3), m (3)], plus B_0 per sensor only if drift proves a problem. Prediction p ← p + v dt, v ← v, Q as above on p and v, Q_m = (0.01 |m|)² per axis per second. Measurement per sensor i: h_i(x) = J^m(r_i) m with r_i = s_i − p, Jacobian rows [−J^r(r_i, m), 0, J^m(r_i)] from Wahlström (30a)–(30b) with the sensor rotation applied, R_i = diag(0.011², 0.011², 0.006²) mT² plus the (0.03–0.05)² systematic floor. R is diagonal, so process the 3N scalar measurements sequentially (Bierman form, *recalled*): each a 9-vector gain, no matrix inverse, and a saturated or missing sensor is skipped by not running its rows. Iterate the update twice at short range. Gate each scalar innovation at 3–4σ; that is also the glitch filter. Cold start stays LM (lattice seed); the EKF replaces the tracking fit. Cost at N = 8: 24 rows × 9 × 9 ≈ 2000 flops per pass, two passes, under 0.2 ms on the V5F; under 1 ms at N = 24. P₀ from the LM covariance for p, (100 mm/s)² for v, (0.1 |m|)² for m.

It buys a consistent update from a frame with one plainly-lit sensor and three faint ones instead of a wild fix to be gated out, free handling of dropouts and saturations, B_0 and moment drift as states, and a covariance right by construction. It costs a second solver to test against LM, divergence when the hand outruns Q (an IMM is the cure at double the cost), and the Gaussian assumption at short range. At good SNR it will match LM, since both bottom out on the same floor; the literature has no EKF beating a converged per-frame LM on a static magnet. Build it as `MagEkf` beside `MagFit`, sharing the field and Jacobian code, run both on the host against the ruler and row-mode recordings, and switch only if it wins on the overhang and weak-signal segments.

### 5.8 Snapping and hysteresis as a prior

**Bubble and area cursors.** Kabbash and Buxton (CHI 1995): an area cursor obeys Fitts' law with the cursor's width. Grossman and Balakrishnan, The Bubble Cursor (CHI 2005, *verified*): radius = min(containing distance to the nearest target, intersecting distance to the second), so exactly one target is always captured and each target's effective width is its Voronoi cell; faster than the point cursor in all 27 conditions (p < .0001), means 1.41 / 1.08 / 0.93 s for object pointing, point cursor, bubble cursor. On a uniform lattice of holes nearest-hole *is* the Voronoi rule, so the useful part is the visual: light the captured hole and let the bubble be the σ ellipse.

**Bayesian selection.** Grossman and Balakrishnan (TOCHI 2005, *verified*) model the endpoint as a bivariate normal with σ(A) = cA. Bi and Zhai, Bayesian Touch (UIST 2013, *verified*): pick `t* = argmin_t ((s − μ)²/(2σ²) + ln σ)` (Eq. 8), add −ln P(t) for unequal priors; 70 % fewer errors than the visual-boundary criterion. Semantic Pointing (Blanch, Guiard and Beaudouin-Lafon, CHI 2004, *verified page*) makes targets bigger in motor space; an absolute device cannot change its CD ratio, so the analogue is hysteresis in the row decision.

**Hysteresis is a prior** (derived tonight, to be checked). For two adjacent rows at 0 and 1, current row with prior P_c, neighbour P_n, equal σ, the Bayesian rule switches at

```
s* = 1/2 + σ² · ln(P_c/P_n)
```

so a margin m past the boundary is a prior ratio k = exp(m/σ²). The current fixed m = 0.1 row is k ≈ 49 at σ = 0.16 rows and k ≈ 1.5 at σ = 0.5 rows: very sticky when the fix is good and almost none when it is poor, which is backwards. A fixed k of 3–5 ("switch when the neighbour is k times more likely") gives m = σ² ln k, clamped to [0.1, 0.45] rows, with σ the scatter of the *smoothed* cursor in rows (what the counted row follows; at rest the 1€ cuts the variance by about α/(2 − α) ≈ 0.03 at α = 0.059, so measure it in the hold-still test). This replaces `RowCounter`'s fixed 0.1 row.

## 6. Pointing: under the tip, or where it points

**Stylus practice.** The sensed point of an EMR pen is a coil above the tip, so a tilted pen reads off to one side; IBM's US 5,239,489 (Russell, filed 1991, *verified page*) calls it "projection error", tan α = dx/dz, tan β = dy/dz, and corrects the reported position by Δx = k sin α (cos β − c), Δy = k sin β (cos α − c) with k and c from the coil geometry and height. Tablets always report the *contact point* and deliver tilt separately (Wacom x/y tilt, Apple altitude/azimuth) for brush shape, never to move the cursor. Our magnet is the tip, so the under-tip mode has no projection error to begin with.

**Ray pointing.** Argelaguet and Andujar's survey (Computers & Graphics 2013, *verified*): "as the selection tool is mainly governed by hand's rotations, its precision is limited to the user's hand angular accuracy and stability. The further away an object is the higher the accuracy is required"; "virtual pointing techniques are less tolerant to noise, as they mainly rely on rotational data"; 60 ms is the maximum latency before interaction degrades (Pawar and Steed, quoted there); users of damping and snapping "tend to complain mainly about flickering effects". Kopper et al. (IJHCS 2010) model distal pointing by *angular* amplitude and width, the lever arm in another form. Whether users want the pointed mode at all is worth testing before it goes in the default menu.

**Geometry.** Tip p = (x, y, z) in the fit frame (z = 0 at the Hall elements), shaft unit vector d continued past the tip (the fitted moment direction, sign fixed once by the probe's build; in `MagLocator` terms d = −shaft), surface plane z = z_s in the same frame. The ray meets the plane at t = (z_s − z)/d_z, cursor c = p + t d, horizontal offset r = t √(d_x² + d_y²) = (z − z_s) tan θ with θ the tilt from vertical. Rules: if t < 0 (tip already at or below the surface, in a hole) use under-tip; if |d_z| < sin 20° (shaft within 20° of horizontal) the ray is useless, use under-tip; cap r at r_max ≈ 2 rows (5 mm) for a hole cursor. (`MagTracker` caps at `MAGTRACK_MAX_REACH_MM 40`, which is right for the LCD's pointer cross and too loose for a hole cursor; the two uses can carry two caps.)

**The frame of z_s, settled.** The first draft and its section notes wrote the surface height in two frames (7.1 mm "above the base PCB" in the menu, the code comments and the LED note; "above the sensor PCB" in the fusion note), and a coder taking any one of them would hard-code the wrong plane. What the firmware measures (verified in source): the fit frame is defined once, in `MagArrayConfig.h` ("origin at sensor 0 … +z up out of the board toward the probe"), every sensor in the table has z = 0, and the `underside` flag only mirrors axes, so **z = 0 in the fit frame is the plane of the Hall elements**, not any PCB surface. And the "surface height" the firmware carries is not a geometric surface at all: `ROWCOUNT_TOUCH_Z_MM 17.5` ("the magnet's height when the probe rests on the breadboard") is learned by the `c` calibration from `fix.rawTip.z` while the probe sits in a hole, as the median over the twelve taps, and pushed into `magLocator.boardZ`, which `MagLocator.cpp` copies into `track.surfaceZ` every frame; `rawTip` is the magnet carried `tipOffsetMm` down the shaft, and with the present `MAGLOC_TIP_OFFSET_MM 0.0` the tracked point is the magnet centre; `MAGLOC_BOARD_Z_MM 17.5` is the same number under a second name. So **z_s = the height, in the fit frame, at which the tracked point rests when the probe is seated in a hole**, measured, never typed. It is the right plane for the pointed cursor, better than the geometric surface: the ray from the hovering tip along the shaft meets it where the tracked point will be once the user pushes in, which is the hole (the seated height varies with tilt as cos θ times tip-offset-plus-insertion, 0.17 mm of height and 0.05 mm of cursor at 15° for 5 mm; ignore it).

Kevin's "7.1 mm above the base PCB" (the shell top above the MainBoard's top surface) is a different quantity in a different frame and must never be assigned to `surfaceZ`; its uses are an a-priori bound for the lattice search (`magFitCoarse` heights 4–110 mm) and a sanity check on what `c` returns. Converting, with z_pcb measured up from the MainBoard's top and z_el the element's height in that frame: z_fit = z_pcb − z_el, z_el = +0.73 mm for a top-side part and −(t_PCB + 0.73) for an underside part (0.73 mm from DS §6.3.2 Fig 6-2, verified from the rendered page; t_PCB = 1.6 mm from `MainBoard.kicad_pcb`, four layers, KiCad's default, so confirm it is the intended fab thickness; ±10 % is absorbed by the calibration). Hence the shell top sits at 7.1 − 0.73 = **6.37 mm** in the fit frame with top-side sensors and 7.1 + 1.6 + 0.73 = **9.43 mm** with underside ones, and the a-priori tip rest height is that plus h_tp, the tracked point's height above the shell top when seated (magnet centre above the tip minus insertion depth), ≈ 9.4 or 12.4 mm for h_tp ≈ 3 mm, both inside the lattice's 4–110 mm.

**Which side (proposed, on the evidence of the MainBoard as laid out on 2026-09-11).** The 600 `WS2812B-1313` of the 16 × 30 block are all on F.Cu (30 columns x = 122.49 … 196.15 mm, 20 rows y = 102.71 … 150.97 mm: twelve hole rows at 2.54 mm with the centre pair 2.54 mm apart and no channel, then four more rows per side at 2.79 / 2.54 / 2.29 / 2.54 mm spacings, §7.1); the 400 1010s of the other experiment are at x = 32.6 … 106.3; all twelve CH446Q are on B.Cu under the *left* block, and **B.Cu under the 16 × 30 block is empty**; no magnetometer is in either schematic. A 1313 is 1.3 mm square, so the top-side interstices are 1.24 mm and the widest row gap 2.79 mm, and a DBV body is 2.9 × 1.6 mm with leads to ~2.8 mm: the sensors cannot go top-side under the breadboard without deleting LEDs. So: **the sensor PCB is the MainBoard (1.6 mm, four layers), the TMAG5273s on its underside beneath the 16 × 30 block, elements at z_el = −2.33 mm in the base-PCB frame, shell top at 9.43 mm in the fit frame.** Its cost by the r⁴ law is in §3.4 (a quarter of the bench error bar at the same pitch); four copper layers are transparent to DC, the Spring Clip6 contacts are bronze, and steel screws, USB shells and Alloy-42 leadframes stay away from the sensors (the bench's RJ45 lesson). No separate sensor board is needed. If Kevin moves the LEDs or adds a sensor board, the conversion above gives the other case in one line.

For the code: the two defines for one quantity (`ROWCOUNT_TOUCH_Z_MM` in `RowCounter.h` and `MAGLOC_BOARD_Z_MM` in `MagLocator.h`, both 17.5) become one, kept in `MagLocator.h` since `RowCounter.h` already includes the locator (`#define ROWCOUNT_TOUCH_Z_MM MAGLOC_BOARD_Z_MM` or direct use), both the `c` line and the `S` line print the name that holds the value, and the fit layer never includes the row-counter header. The `#define` is only the boot value that `c` printed last time.

**Error propagation.** With independent errors σ_tip on the tip position and σ_θ (radians) on the shaft angle, at height h = z − z_s:

```
σ_pointer² = σ_tip² + h² σ_θ²        (small tilt)
σ_pointer² = σ_xy² + (h sec²θ)² σ_θ² + tan²θ σ_z²    (tilted: r = h tan θ, ∂r/∂θ = h sec²θ, ∂r/∂h = tan θ)
```

At h = 30 mm, σ_θ = 3° (0.052 rad) adds 1.6 mm and 5° adds 2.6 mm, one row; at h = 5 mm the same angles add 0.26–0.44 mm. So the pointed mode costs about a row of noise at full hover height and nothing near the board, and the angle must be filtered harder than the position. Position and moment are jointly estimated, so s²(JᵀJ)⁻¹ has the cross term; use it if exposed, else the sum is an upper bound. The virtual plane also kills the other failure: continuing the ray *through* the breadboard to the element plane would land z_s · tan θ further on (9–12 mm · tan θ on V6, 17.5 on the bench): 0.7–0.9 rows at 10° and 2–3 rows at 30° on V6.

**Blend.** Three settings: UNDER, POINTED, AUTO. In AUTO the pointed weight ramps from 0 at h = 3 mm to 1 at h = 15 mm, and fades back toward under-tip when σ_pointer exceeds 1.5 mm. The cursor is c = p + w r, so its variance is the tip's (common to both ends of the blend) plus w² times the ray term; the first draft's `sqrt((1−w)² σ_tip² + w² σ_ptr²)` with σ_ptr² already containing σ_tip² under-counted the common term (at w = 0.5 it gave half of σ_tip²):

```c
float h = p[2] - zs;  float cx = p[0], cy = p[1];  float w = 0;   // p, zs in the fit frame; zs = magLocator.boardZ, calibrated
if (mode != UNDER_TIP && h > 0 && d[2] < -0.34f) {          // pointing down, > 20 deg from horizontal
  float t = -h / d[2];  float rx = t*d[0], ry = t*d[1];  float r = sqrtf(rx*rx + ry*ry);
  if (r > rMax) { rx *= rMax/r; ry *= rMax/r; }
  float sigRay2 = h*h*sigTheta*sigTheta;                    // the ray's own term (small tilt; add tan²θ σ_z² when tilted)
  float sigPtr = sqrtf(sigTip*sigTip + sigRay2);
  w = (mode == POINTED) ? 1.0f : clamp((h - 3.0f)/12.0f, 0, 1);
  if (sigPtr > 1.5f) w *= clamp((3.0f - sigPtr)/1.5f, 0, 1);
  cx += w*rx;  cy += w*ry;
  cursorSigma = sqrtf(sigTip*sigTip + w*w*sigRay2);         // σ_c² = σ_tip² + w² h² σ_θ²
} else cursorSigma = sigTip;
// then 1€ on (cx, cy); row snap with m = clamp(sigRowSmoothed² ln 4, 0.1, 0.45) rows
```

Order per frame: sensor frame (minus baseline and, on V6, the known-input K·I of §2.11) → LM seeded from x̂⁻ under an evaluate budget → at most one leave-one-out refit, only if the misfit is high → Huber gate, floor, reinit → coast decision → blend → 1€ on cursor and shaft → Bayesian snap with σ-scaled hysteresis → LED ring of radius 2 cursorSigma (grey when coasting).

## 7. LEDs: position and uncertainty on the breadboard grid

### 7.1 What is being drawn, and on what

The renderer receives, at 100 Hz, a cursor on the surface plane, a covariance, a misfit and a shaft direction. `RowGrid` maps a board-frame point to breadboard millimetres: `along` in rows (2.54 mm) and `acrossMm` from the channel centre line. The bar is checked (truth inside 1σ 66–71 %, inside 2σ 92–97 % in simulation) but under-reports repeatable error, so the renderer floors σ; and σ is per axis in the board frame while the LED table lives in the breadboard frame through the fitted affine map, so the covariance rotates with it.

**V5** (`LEDs.h`): 300 breadboard LEDs at five per row, rails at indices 300–399 as `railsToPixelMap[4][25]`, plus 145 "top" LEDs on a second strip. Rows 2.54 mm apart; innermost holes of the two halves 7.62 mm apart centre to centre (`ROWGRID_INNER_HOLE_MM 3.81`), so the empty channel is 5.08 mm; hole positions across the channel are ±(3.81 + k · 2.54), k = 0..4. Twenty-five rail LEDs along 30 rows is a 3.048 mm pitch if the rails span the same length as the rows; the V6 MainBoard's 400-LED 1010 block, which replicates the V5 layout (`MainBoard.kicad_pcb`, *verified*: 10 hole rows of 30 at 2.54 mm with the 7.62 mm channel, 4 rail rows of 25 at 3.048 mm, x = 32.6 … 106.3), confirms it: rail LEDs do not line up with rows, and the table must carry millimetre coordinates per LED rather than (row, column). **V6**: a 16 × 30 grid, "6 + 6 holes per row plus two rail LEDs each side", 480 LEDs, one per hole. The MainBoard as laid out on 2026-09-11 (`MainBoard.kicad_pcb`, *verified* by parsing the `WS2812B-1313` footprint positions) has **600** of them on F.Cu, 30 per row along x = 122.49 … 196.15 mm (29 × 2.54) in **20** rows across: twelve hole rows at 2.54 mm from y = 112.87 to 140.81 with the centre pair at 125.57 / 128.11 (2.54 mm apart, **no channel**: six holes to the centre, centre line at y = 126.84), then on each side a pair of rows 2.54 mm apart whose inner member is 2.79 mm outside the last hole row (±16.76 and ±19.30 mm from the centre line), and beyond that a second pair, 2.29 mm further out (±21.59 and ±24.13 mm). So the twelve hole columns are contiguous at 2.54 mm with the innermost pair at ±1.27 mm as assumed, but there are *four* extra rows per side, not two, and the outer rows are not on the 2.54 mm lattice across. Which of the eight outer rows are the two rail LEDs per side Kevin described, and what the other four are, cannot be told from footprints; the schematic or Kevin settles it, and the V6 table (millimetres per LED, as V5's) is built from the file either way. If all 600 are one chain the frame is 18 ms, not 14.4, which only strengthens the ≥ 2-chain rule in §7.3.

### 7.2 What the literature says about showing "here, but unsure"

- **Area and bubble cursors** (§5.8): the *activation* radius (which hole a click selects) can be nearest-hole while the *drawn* radius says how sure; the bubble cursor is the precedent for drawing one radius that is not the other.
- **Bayesian selection carried through the UI**: Schwarz, Hudson, Mankoff and Wilson (UIST 2010) and Schwarz and Hudson (UIST 2011, *abstract*) carry a distribution over interpretations instead of collapsing it early. Our fit already is a Gaussian posterior; lighting each hole with the probability mass inside it is that picture drawn literally.
- **Uncertain Pointer** (CHI 2026, arXiv:2602.13433, Microsoft Research, *summary of the fetched paper*), the most directly applicable study: a 25-pointer design space over signifiers (colour, size, opacity, text), archetypes and uncertainty types; in the graded-confidence study (n = 40) external pointers ranked highest, boundary worst, text gave the lowest error, and "opacity preserved visibility better but conveyed uncertainty less effectively". On an LED grid there is no text and every hole is a candidate, so size (a wider patch) has to carry the message and intensity confirms it.
- **Gaze cursors**: Tobii's guidance (*verified*) is "Only visualize the effect of gaze. Don't visualize the gaze position", except that "showing a spotlight effect around the gaze position can provide feedback … The highlighted region should be large enough to encompass the user's actual gaze." Hassoumi et al. (J. Eye Movement Research 2018, *abstract*) draw heat maps whose kernel size follows the per-sample uncertainty. That is the per-fix-σ Gaussian splat.
- **Uncertainty visualisation**: Correll and Gleicher, Error Bars Considered Harmful (IEEE TVCG 2014, *abstract*): hard-edged encodings create within-the-bar bias, gradient plots do not; a hard 2σ ring of LEDs would carry the same bias, a soft fall-off does not. Correll, Moritz and Heer, Value-Suppressing Uncertainty Palettes (CHI 2018, *abstract*): high uncertainty suppresses the value encoding, fewer and duller colours, "encourages more cautious decision-making". Hullman, Resnick and Adar, Hypothetical Outcome Plots (PLOS ONE 2015): animating draws from the distribution is readable by untrained viewers; on LEDs that is a slow low-amplitude shimmer, to be used sparingly if at all.
- **Radar and sonar**: an area of uncertainty around a contact grows without new contact information (US 9,292,971, radartutorial.eu, *summary*); the cursor should do the same during dropouts.

So: size carries the message and intensity confirms it; soft edges avoid false precision; uncertainty dims and desaturates rather than adding colour; a spotlight big enough to contain the truth is acceptable where a pointer is not; hue is read categorically and stays reserved for other meanings; dither reads as "noise" and is for the far-field case at most.

### 7.3 Rendering a point into a coarse grid

**Sub-pixel position.** Kriegsman's anti-aliased FastLED bar (2013, *verified*) positions in 16ths of a pixel and splits the end pixels' brightness (`firstpixelbrightness = 255 − frac·16`); JumperlOS's `LogoRing` does the same on the ring. In two dimensions that is bilinear splatting into four LEDs; a Gaussian splat generalises it, and with σ ≥ ~0.5 mm the Gaussian *is* the anti-aliasing.

**Gamma and the dim end.** CIE 1976: L* = 116 f(Y/Yn) − 16, f(t) = ∛t above δ³ else t/(3δ²) + 4/29, δ = 6/29; γ ≈ 2.2–2.8 approximates it. The catch with WS2812-class parts (Mountain Lizard's measurements, *verified*): "for about the first 20 input values, the output ramps up considerably slower than linearly" (code 3 → 0.001, code 10 → 0.01 of full), so a plain gamma table loses the dim tail of a Gaussian; they publish a compensated 256-entry table. FastLED's temporal dithering (*verified*) gets fractional levels by alternating codes between frames and warns of visible flicker at low refresh rates; the WS2812B's own PWM is ≥ 2 kHz, so dithering is limited by *our* frame rate. At low codes the three channels quantise differently and the hue drifts (FastLED issue #4042), which is why a dim cursor should be near-neutral.

**Timing.** WS2812B: 800 kHz, 30 µs per LED, reset ≥ 50 µs (280 µs in later revisions). V6: 480 × 30 µs = 14.4 ms per frame on one chain (18 ms for the 600 the 2026-09-11 MainBoard carries, §7.1). JumperlOS today: `ledClass::show()` notes 13 ms for 300 + 145 LEDs, two strips sent with asynchronous DMA, a frame *dropped* rather than waited for when DMA is busy; treat the achievable full-frame rate as ~60–70 Hz on one chain and lower under load. Deber, Jota, Forlines and Wigdor, How Much Faster is Fast Enough? (CHI 2015, *verified*): latency JNDs of 11 ms for direct dragging, 55 ms indirect dragging, 69 ms direct tapping, 96 ms indirect tapping, improvements as small as 8.3 ms noticeable, Ng et al. (UIST 2012) at 6 ms for direct dragging; Jota et al. (CHI 2013, *summary*): dragging performance degrades above ~25 ms. A probe over LEDs that follow it is the direct case. On V6 the fit runs on the CH32H417 and the frame is composed on the RP2350 (§9.5); the FSMC→PIO link itself costs ~1.5 µs per message and adds a **phase wait** for the next cursor tick that finds the WS2812 DMA free. With T_dma = N × 30 µs, a 10 ms tick, ~6 ms for sampling + fit and 0.5 ms of PWM, the mean chain is **24–31 ms on one 480-LED chain** (20 ms period, 10 ms mean wait, 7.2–14.4 ms of transmit to the cursor's LED; worst ~40), **15–19 ms on two chains of 240** (worst ~28) and **11–13 ms on four of 120 with a 5 ms tick** (worst ~20). One chain fails Jota's 25 ms line on average, two clear it, four approach the JND; so the V6 board routes the 480 breadboard LEDs as at least two chains, preferably four, on separate PIO state machines (JumperlOS already drives two strips). The lead is not a constant "v · 20 ms": core 1 extrapolates by the measured age, `age = age_us + (t_now − t_arrive) + chainOffset(pixel)`, clamped at 40 ms so it never extrapolates past a dropout and scaled down at low speed, which is LaValle et al.'s Method 2 for the Rift (ICRA 2014, *verified*: "Assume the currently measured angular velocity will remain constant over the latency interval", with "simple smoothing filters in the estimation of current angular velocity" and "We also shorten the prediction interval for slower rotations"; over 20 ms their average error fell from 1.46° to 0.19°). Here v is the CV Kalman state, already smoothed, and with two chains the mean lead is ~17 ms and follows the real chain.

### 7.4 The design

**Coordinate-indexed LED table**, one per board, built at boot from the existing maps (`rowColumnToPixelIndex`, `railsToPixelMap`, `board::currentBoard().caps.ledsPerRow`), never re-derived:

```c
struct CursorLed { float along_mm; float across_mm; uint16_t pixel; uint8_t isRail; };
static CursorLed g_led[600];  static int g_ledCount;      // 600 on the 2026-09-11 MainBoard; 480 if the extra rows go
// V6 (MainBoard 2026-09-11, measured): hole columns c = 4..15 -> across = (c - 9.5) * 2.54 (innermost pair ±1.27, no channel);
//   outer rows at ±16.76, ±19.30, ±21.59, ±24.13 mm (c = 3,2,1,0 and 16..19), which of them are rails: from the schematic.
// V5: holes across = ±(3.81 + k*2.54), k = 0..4; rails at 3.048 mm along-pitch, across from the board file.
// along_mm = (row - 1) * 2.54 for rows 1..30, the same for 31..60 on the other side.
```

plus a per-board 2-D index `g_grid[ROWS][COLS]` (COLS = 20 across on the 2026-09-11 V6 layout, 16 if the extra rows go; −1 in the V5 channel gap) so the render loop visits a σ-box, with V5's rail LEDs walked in a second pass by `isRail`.

**Covariance in the breadboard frame.** `RowGrid` is the affine map along = ax x + ay y + a0 (rows), across = cx x + cy y + c0 (mm). With A = [[2.54 ax, 2.54 ay], [cx, cy]] and Σ_board = diag(σx², σy²):

```
C = A Σ_board Aᵀ ;  C += diag(σ_min²), σ_min = 0.8 mm ;  C⁻¹ = (1/det) [[C22, −C12], [−C12, C11]]
```

**Peak from area, not fixed**, so the light budget says "sure": with σ₀ = 1.3 mm (about half a pitch)

```
peak = P_max · min(1, σ₀² / √det C) ;  peak = max(peak, 0.12 P_max) ;  peak *= confidence(misfit)
confidence = clamp(1 − (misfit − 0.05)/0.35, 0.25, 1)      // 0.03 clean, 0.3 rough, 0.40 the limit
```

At σ = 0.8 mm the neighbours 2.54 mm away get exp(−½ (2.54/0.8)²) ≈ 0.006 of the peak, one hole; at 1.27 mm they get 0.135, a soft cross; at 5 mm the patch is ~8 × 8 LEDs at 12 % peak, the flashlight tightening to a point without any mode logic.

**Render loop** (RP2350 core 1, as a *cursor plane*: `baseFrame[480]` is a copy of the last full compose, each tick copies base → out for the σ-box, adds the splat and shows; suppressed under `LED_MENU`, `LED_GFX` and the user toggle; never posts `requestLedShow`; §9.5). Its input is the newest complete `ProbeCursorMsg`, extrapolated by its age; on the single-chip test bed the same state comes from the tracker directly:

```c
void renderProbeCursor(const ProbeCursorState* s, uint32_t* frame) {   // s: from the message, age applied
    if (!s->valid) return;
    float ia = 3.0f*sqrtf(s->C11), ic = 3.0f*sqrtf(s->C22);          // 3-sigma box
    int r0 = rowOf(s->along - ia), r1 = rowOf(s->along + ia);         // clamp 0..29
    int c0 = colOf(s->across - ic), c1 = colOf(s->across + ic);       // clamp 0..COLS-1
    for (int r = r0; r <= r1; r++) for (int c = c0; c <= c1; c++) {
        const CursorLed* L = &g_led[g_grid[r][c]];
        float da = L->along_mm - s->along, dc = L->across_mm - s->across;
        float q = s->Ci11*da*da + 2.0f*s->Ci12*da*dc + s->Ci22*dc*dc;  // Mahalanobis^2
        if (q > 9.0f) continue;
        float w = s->peak * expLut(q);                                  // 64-entry LUT of exp(-q/2)
        w += s->tailWeight(da, dc); if (w > 1.0f) w = 1.0f;             // tilt tail, see below
        uint8_t code = g_gamma[(int)(w * 255.0f)];                      // low-end compensated
        if (code < 8) continue;                                         // below the chip's floor
        frame[L->pixel] = blendAdd(frame[L->pixel], scale(warmWhiteForHeight(s->height_mm), code));
    }
}
```

At most ~19 × 16 LEDs for σ = 8 mm, ~300 lookups, well under a millisecond on either MCU.

**The rest of the rules.** Smooth `along` and `across` with the 1€ (§5.5) *before* rendering (on the CH32, before the message), add the latency lead at the consumer (§7.3), and low-pass σ separately at 5 Hz, asymmetrically (grow at once, shrink through the filter) so one bad frame does not snap the patch open and shut. Far away: fade the peak with confidence and let σ carry the size; when the estimate lies outside the LED rectangle clamp it to the edge and draw the Gaussian there, so a soft edge glow says which side the probe is on, optionally with a one-LED tick breathing at 1 Hz. Dropout: keep the last position, inflate σ (σ² += q Δt², q ≈ (300 mm/s)²), fade, release after ~300 ms, the area-of-uncertainty rule. Hole selection: the hole with the largest Gaussian mass, switching only when a new hole's mass exceeds the current one's by 20 %; the patch does not move with the hysteresis, only the selection, which goes to `brightenNet()` on a click so it looks like a probe touch. Height as colour temperature: warm white (placeholder 255, 180, 110 pre-gamma) at contact to cool white (200, 220, 255) by 25 mm up, both low-saturation so they never collide with the net palette; height must not go into brightness (spoken for by σ) or hue. Tilt as a tail: when the shaft is > 15° from vertical and σ_a, σ_c < 1.5 mm, two weights along −d̂ at 1 and 2 pitches behind the tip, 0.35 and 0.12 of the peak, σ = 0.8 mm each; skipped when the patch is wider than a hole. Temporal dithering only once the breadboard runs as ≥ 2 chains at ≥ 120 Hz. The cursor is an overlay blended last (`blendColors`, `tintPixel`), its peak above the palette (~0x80–0xC0 after gamma; palette values live at ~0x30), it avoids saturated red (the `warnNet` short animation) and never brightens nets by itself. `PCBEXTINCTION` and the per-hue `PCB…SHIFT` values in `LEDs.h` are marked unused and never wired in; measure the white point once through the real V6 plastic before choosing the warm/cool endpoints. When `show()` completes, the render code publishes the frame's per-chain current sum `I_chain = Σ (0.35 mA + 5 mA (R + G + B)/255)` for the known-input correction of §2.11 (over the link with the ack on V6), so the tracker subtracts the right frame's field; the cursor patch itself is part of that sum, which is what keeps its 6 µT at 3 mm from biasing the fit toward itself. Test before tuning: log (along, across, C, misfit) at 100 Hz, replay into a host-side renderer that prints frames as ASCII, tune σ_min, σ₀, P_max, the 1€ and the tail there.

## 8. On-screen UI and camera control

### 8.1 What exists

`Console` (5 ms service) dispatches single characters from a `Stream` through a fixed table of 32 `{key, help, handler}` entries that modules fill from `begin()`; numeric arguments (`R30`, `t12`) go through a *blocking* `consoleReadNumber()`. Registered tonight: `? X`, `m z p f i b` (array), `d l o k K t T` (locator), `r c R C h` (rows), `v` (camera). `MagView` draws the scene into a `GFXcanvas16(240,240)` in one 3 ms tick and pushes 15-row bands on the following ticks (~18 fps). Its camera is `{target, zoom (px/mm at target), cameraDistance, yaw, elevation}`; from `project()` (checked numerically) the forward vector is **f = (−cosEl sinYaw, cosEl cosYaw, −sinEl)**, screen-right r = (cosYaw, sinYaw, 0), screen-up u = (−sinYaw sinEl, cosYaw sinEl, cosEl), eye = target − cameraDistance · f, and roll is structurally zero. The fix carries `shaft` (unit, up the probe, shaft.z ≥ 0), `tip` and `pointer`; pointing direction d = −shaft.

JumperlOS's `OLEDStream` strips ANSI, collapses newlines, expands tabs and hands text to an eight-line static buffer with word wrap; its menu is data-driven from `menuLines[150]` but `getMenuSelection()` is a blocking loop; `EncoderClickTracker` classifies PRESS / CLICK / HOLD (500 ms) / LONG_HOLD (1500 ms) / HOLD_RELEASE with 30 ms debounce, hold-to-back repeats every 800 ms, and a no-input timeout returns to the remembered position. "Compatible in spirit" means keeping that vocabulary and those idioms, and a registry modules fill from `begin()` like `consoleAddCommand`, but as a non-blocking state machine inside a `Service`, because a blocking loop starves the 100 Hz read.

### 8.2 Patterns and numbers

u8g2's `userInterfaceSelectionList` / `InputValue` / `Message` are blocking but set the conventions (cancel returns 0, a value is written only on confirm); its successor MUI (*verified*) is the shape to copy: forms and fields in fixed tables, `nextField`/`prevField`/`sendSelect`/`gotoForm`, no malloc, under 0.5 KB. LVGL (*verified*, `lv_conf_template.h`): long press 400 ms, repeat 100 ms, double-click 400 ms, refresh 33 ms; its `lv_menu` is now deprecated in favour of plain containers, itself a lesson that a page stack plus a title line is all the structure a menu needs. Marlin (*verified*): encoder acceleration at 30 and 80 steps/s, `LCD_TIMEOUT_TO_STATUS 15000` ms. Windows keyboard repeat: delay 250 ms–1 s, rate 2.5–30/s, default ~500 ms at ~30/s (*recalled*). Ganssle's debouncing measurements (*verified*): 16 of 18 switches averaged 1.6 ms of bounce with a 6.2 ms maximum, one outlier 157 ms; "pick a debounce period in the 20 to 50 msec range", poll every 1–5 ms, "a 50 msec response seems instantaneous". A 4-way nav switch is four snap domes, each debounced as its own button; a joystick's centre switch is often worse than the domes.

Conventions worth copying (handhelds, cameras, printer LCDs, *recalled*): lists wrap; the selected row is an inverted bar; a one-level breadcrumb title (40 characters at 6 px per glyph is a 240 px line); a scroll indicator only when the list overflows; Back keeps the parent's cursor; `>` marks a submenu; toggles flip inline; enums cycle on left/right; numeric items open an editor where hold accelerates, select commits, back restores the old value; destructive actions ask yes/no; a no-input timeout closes to the live view but remembers the position.

**Joystick.** Scaled radial dead zone (Sutphin, *verified*; XInput uses 0.24/0.27 of full scale and suggests cubing for fine control): `v = normalize(v) · ((|v| − dz)/(1 − dz))` so the output starts from zero at the edge. Betaflight's expo (*verified*): `out = in · |in|³ · expo + in · (1 − expo)`; at expo 0.5 half-stick gives 31 % of full rate. A digital 4-way layer with hysteresis (engage at 0.60, release at 0.35) feeds four virtual buttons so a held stick repeats like a held key.

### 8.3 Camera

**Orbit / pan / zoom** in MagView's parameterisation: yaw += rate · stickX · dt, el = clamp(el + rate · stickY · dt, 5°, 89.5°); pan moves target along screen-right and the board-plane projection of screen-up scaled by 1/zoom so it feels constant in pixels; zoom multiplies by exp(±rate · dt); presets top (0, 89.5), side (el 8), iso (−35, 30), plus the existing sway and spin.

**Interpolation.** Driscoll (*verified*): frame-rate-independent damping is `a = lerp(a, b, 1 − exp(−λ dt))`, the cheapest smoother, for continuously tracked things (follow target, POV direction). For discrete jumps (presets, entering POV) a critically damped spring has velocity continuity and no overshoot: Juckett (*verified*) gives the ζ = 1 closed form; Unity's `SmoothDamp` (*verified* from UnityCsReference) is the production version, `omega = 2/smoothTime`, a cubic rational approximation of the exponential, a clamp on the change and an overshoot snap, and `SmoothDampAngle` wraps the difference first so 350° → 10° travels +20°, not −340°. Zoom is interpolated in log space. Quaternion slerp is unnecessary: roll is fixed and two angles can be handled one at a time.

**Follow mode.** The target chases the tip or pointer through an exponential lag (τ ≈ 150 ms), a dead band of ~2 mm so fit noise does not shake the scene, and a slew limit (300 mm/s) so a fix jump after a dropout slews rather than snaps; third-person camera practice calls these lag, dead zone and slew limit and uses all three (Haigh-Hutchinson 2009, *recalled*).

**POV mode.** Oculus's best practices (*verified* PDF): rotating or moving the horizon "can be discomforting"; "make accelerations as short (preferably instantaneous) and infrequent as you can"; "have accelerations initiated and controlled by the user"; no head bobbing; "user and camera movements should never be decoupled". An analogy for a 240 px panel, but the translation holds: keep roll zero, smooth the *direction* harder than the position because hand tremor on the shaft is amplified into whole-scene rotation, and clamp the eye height to a floor. Inverting f gives the camera from the shaft:

```
el  = asin(−d.z) = asin(shaft.z) = 90° − tiltDeg
yaw = atan2(−d.x, d.y) = atan2(shaft.x, −shaft.y)
```

(checked: (0, 89.9), (−25, 32) and (120, 10) round-trip). Two singularities: the shaft straight down makes yaw undefined, so below a tilt threshold hold the last good yaw (8° in, 12° out, and let the joystick steer `heldYaw` meanwhile); a nearly horizontal shaft is clamped to el ≥ 15° so the view keeps looking at the board. A camera *at* the tip sees nothing when the tip is on the board, so the eye sits `backOff` mm up the shaft looking along −shaft, with target = eye + dist · f. The default back-off is max(tipOffset, 10 mm), not "the tip offset": `MAGLOC_TIP_OFFSET_MM` is 0.0 today (the tracked point *is* the magnet centre, §6), so "at the magnet" would put the eye at the tip, which the previous sentence says sees nothing; `dist` doubles as the focal length, F = zoom · dist pixels, horizontal FOV = 2 atan(120/F): dist 40 mm at zoom 3 px/mm gives 90°. Direction smoothed with τ = 250 ms, position with 80 ms. When the fix drops out the camera holds; entering POV from a preset uses the 0.35 s spring instead of a cut.

**Near-plane clipping is a required change to `MagView`, not an option.** `project()` clamps range to 20 mm but never culls; in POV with dist = 40 the board beside and behind the eye has depth < −dist, the clamp turns that into a positive range and the point projects mirrored, so `line3d` segments crossing the eye plane streak across the panel. `line3d` must compute depth for both endpoints, drop a segment when both are behind −dist + 5 mm, and otherwise move the offending endpoint to the crossing by interpolating in board space before projecting. The trail, grid lines and board outline are the segments most likely to straddle it.

### 8.4 The design

**Menu**, every console command an item, typed; `(x)` is the key it mirrors, *new* marks options the other sections add; 20 s timeout back to View, cursor remembered:

```
View  screen   Log  screen
Camera >  Preset enum fixed|sway|spin|top|side|iso|follow|POV (v); Follow lag ms 0..1000/50 = 150;
          POV smoothing ms 50..1000 = 250; POV eye back-off mm 0..40 = max(tip offset, 10); Orbit rate deg/s 30..360 = 120; Reset view
Probe >   Latest fix -> Log (l); Stream fixes CSV toggle (d); Tip offset mm 0..60 (t); Magnet angle deg 0..90/5 (T);
          Learn strength (k); Forget strength confirm (K); Orientation check (o); Cursor mode enum under|pointing|blend new;
          Tip rest height mm 0..60/0.1 = calibrated (fit frame; shows "(calibrated)" or "(typed)") new;
          Measure rest height action "put the probe in a hole, hold still" (one tap, the first-tap rule of c) new;
          Motion gate toggle new; Coast on dropout ms 0..1000/50 new
Rows >    Row mode toggle (r); Calibrate rows confirm (c); Anchor row 1..60 (R); Forget anchors confirm (C); Hold-still test -> Log (h)
Array >   Status -> Log (m); Re-zero baseline confirm "magnet away?" (z); Power-cycle sensors confirm (p);
          Stream fields CSV toggle (f); Identify sensor toggle (i); Bus check -> Log (b)
LEDs >    Probe cursor toggle; Uncertainty halo toggle; Brightness % 0..100/5; Layout enum V6 6-hole | V5 5.08 gap   (all new)
System >  Services -> Log (X); Help -> Log (?); Inputs screen (raw joystick/buttons); Menu timeout s 5..120
```

Registry, filled from each module's `begin()` next to its `consoleAddCommand` calls so the two stay in step. On V6 the panel, the nav switch, the joystick and the buttons hang off the RP2350 (where `OLEDStream`, `Menus.cpp` and MicroPython already draw, and the V5 encoder already lives) while the probe modules run on the CH32H417, so an item's value is not always a pointer in the same address space; each item says where it lives:

```c
enum ItemKind { ITEM_ACTION, ITEM_TOGGLE, ITEM_INT, ITEM_FLOAT, ITEM_ENUM, ITEM_SUBMENU, ITEM_SCREEN };
enum ItemHome { LOCAL, REMOTE_CH32 };
struct MenuItem { const char* page; const char* label; ItemKind kind; ItemHome home; char consoleKey; bool confirm;
    bool* flag; int* ival; float* fval; int lo, hi, step; float flo, fhi, fstep;   // LOCAL only
    const char* const* names; void (*onChange)(void); void (*onAction)(Stream* out); float remoteValue; };
static MenuItem items[MENU_MAX_ITEMS = 64]; static int itemCount;
bool menuAddItem(const MenuItem& it);
```

Actions get the tee stream as `out`, so their text lands in Log; INT items get their own editor and call the same setter the handler calls, bypassing `consoleReadNumber`. Remote items are filled from the CH32's registry announce at link-up (`{key, help, kind, lo, hi, step, current}` for every console command), the RP2350 lists them under `Probe >` / `Array >` / `Rows >`, an edit sends `SET key value` over the console channel and shows the acked value, and an action sends the key and the CH32's reply text lands in the log tee (§9.5). On the test bed every item is LOCAL and the link is the loopback back end.

**Input state machine.** One `ButtonTracker` per physical or virtual control: DEBOUNCE 20 ms, HOLD 500, LONG_HOLD 1500, REPEAT_DELAY 400, REPEAT 80, REPEAT_FAST 40 after 2 s (value editors only); events PRESS / CLICK / HOLD / LONG_HOLD / REPEAT / RELEASE, a repeated key never also clicks, and a click is a short press reported on release. Nav U/D/R and the joystick's four digital directions poll with repeat and act on PRESS and REPEAT; nav L, centre, joystick press, A and B poll without and act on CLICK / HOLD / LONG_HOLD, so nav-L backs out one level per click and one per 800 ms while held. Value editors accelerate Marlin-style: ×1 for the first 10 repeats, ×10 after, ×100 after 40. Joystick: dead zone 0.22, expo 0.5, digital in/out 0.60/0.35, `adcMid` calibrated from the first 20 frames at boot (the stick is centred at power-up), half-span a setting. A UI `Service` at 5 ms polls everything; the consumer is a screen stack `{VIEW, MENU, EDIT, CONFIRM, LOG, INPUTS}` six deep with a cursor and top per level.

Mapping (proposal; the two buttons' roles are the open question): **A** click = open menu / select, A hold = jump to Log; **B** click = back (keeps the parent cursor), B hold = close to View. Nav U/D move with wrap; R or centre = enter/edit/toggle; L = back. In View: joystick = orbit (120°/s × x, 90°/s × y), A + joystick or nav = pan, centre = cycle preset, joystick click = toggle POV, joystick hold = reset view. In Edit: U/D or joystick digital step, the 0.22–0.60 analog band adds rate × step × y per second for fine sweeps, centre commits, L/B cancels and restores. In Log: U/D scroll a line, repeat scrolls, hold fast-scrolls, R = page down, centre = jump to newest and re-arm follow.

**Serial log.** A ring of 64 lines × 40 columns (2.6 KB) with head, count, a `view` offset from the newest and a `follow` flag; a `TeeStream : public Stream` whose `write` goes to `Serial` and the ring and whose `available/read/peek` forward to `Serial`, given to `console.begin(&tee)` so every existing `out->println()` in every module lands on the LCD with no module change. ANSI stripped as `OLEDStream::write` does; wrap at write time; the in-progress line always terminated; scrolling up clears `follow`, returning to the bottom re-arms it; 26 rows drawn from `head − view` backwards into the existing canvas through the same band push, a title line showing "-12" when not following, a scroll bar at x = 236. The CSV streams (`d`, `f`) will flood it: show "streaming, hold centre to stop" and put those two toggles in the View status line.

**Budget.** Per 10 ms UI tick: eight `btnPoll`s, one `joyUpdate` (one sqrt), and either five `smoothDamp`s with two trig pairs or `povGoal` (one exp, one asin, one atan2, four trig): well under 50 µs. Memory: items 64 × ~48 B ≈ 3 KB, log 2.6 KB, trackers and camera < 200 B; the 115 KB framebuffer already exists.

**Open questions for Kevin.** Which button is Back, and whether A-hold = Log is right or Log belongs in the preset cycle; joystick ADC resolution and whether its centre is a GPIO or on a resistor ladder; whether nav-centre and joystick-press are interchangeable; and whether `pointer` or `tip` is the follow anchor by default (the pointer stays on the board and is calmer; the tip is what the hand is doing). From the other sections, the proposals that need his yes: the V6 probe magnet (the 4 × 6 mm N52 rod, §4), the sensors on the MainBoard's underside with the fit frame at the elements (§6), the CH32-owns-the-tracker split with the 34-byte message (§9.5), at least two LED chains in the board file (§7.3), and the layout rules for the LED trunks and the switchers (§2.11).

## 9. Recommended architecture and build order

*Where the tree stands against this section, as of the morning of 2026-09-18:* built and flashed overnight from the sections above are the lattice search and the lattice-first cold start (§4), `MagTracker` with the gate, the collinear-outlier restart, coasting and the 1€ cursor (§5, §6), speed-adaptive field smoothing (the soak found the weak-signal smoothing lagging a moving hand by 8 mm; α now rises with the track's speed), the baseline dipole-pollution check and the slow drift while nothing is read (§4.6), `MAG_TEMPCO 00` (§2.5), the LED renderer with the V6 and V5 tables (§7), and the menu / log / camera UI (§8). The rest of the table below - the two-level Q, the χ² presence test, the SET_COUNT gate in place of the median, per-axis weights, ±80 mT, triggered sampling, the AUTO blend, σ-scaled row hysteresis - is still to do, in the order of §9.4. `docs/wireless-probe-sensing.md` has the bench-facing account of what was built and how it was tested.


### 9.1 Data flow, per frame (10 ms nominal; dt measured)

```
TMAG5273 ×N (32×, ±80 mT, tempco 00, XYZ, T off, continuous now → triggered later)
  │  1-byte reads, SET_COUNT/RESULT_STATUS/DIAG gate, duplicates dropped, per-axis 3σ range check
  │  frame stamped from the data: mean micros() of its reads and their count (phase-locked lastFrameUs)
  ▼
frame average per sensor (raw σ ≈ 0.011 / 0.011 / 0.006 mT)  ── raw-frame χ²(24) presence test (T > 43 ×3 on, < 35 off; per-axis σ)
  │  minus live baseline (per-axis IIR, α = 1 − exp(−dt/30 s), updated only while absent; dipole-pollution check before any re-zero)
  │  minus K·I, the board's own field from the known LED-chain and rail currents (V6, §2.11)
  │  per-sensor 3-gain (or 3×3) + offset calibration from magcal
  ▼
MagFit: warm LM seeded from the track's prediction x̂⁻ under an evaluate budget; lattice search (magFitCoarse), one slab per frame, on a cold start
  │  per-axis weights 1/σ²; known |m| when learned; σ = √diag s²(JᵀJ)⁻¹; misfit; at most one leave-one-out refit when the misfit is high
  │  → valid fix (misfit ≤ 0.40, bar ≤ 15 mm) | rough fix (≤ 0.7, ≤ 60 mm, σ ≥ 3 mm) | none
  ▼
MagTracker: three [p, v] CV Kalman axes, T = dt, Huber-weighted χ² gate with 0.5 mm floor, unconditional speed bound, agreeing-reject reinit, coast
  │  shaft: separate 1€ (0.5 Hz, β 0.005 on tip speed), renormalised; flip confirmation
  │  tip = magnet − tipOffset · shaft
  ▼
cursor on the plane z_s (fit frame, the calibrated tip rest height): UNDER | POINTED | AUTO blend, reach cap 5 mm (holes) / 40 mm (LCD pointer)
  │  σ_cursor² = σ_tip² + w² h² σ_θ²; 1€ on (cx, cy) (1 Hz, β 0.03–0.045)
  ▼
RowGrid affine map → (along, across, C)  → RowCounter snap with m = clamp(σ² ln 4, 0.1, 0.45) rows → selected hole
  ═══ CH32H417 ▲ (array, fit, track, cursor, rows)  │  FSMC→PIO link, 34 B ProbeCursorMsg per fit on FMC_A0 = 1  │  ▼ RP2350 (frame, nets, LCD, menu) ═══
  ├─► LED renderer (RP2350 core 1, cursor plane over baseFrame): age-extrapolated position, Gaussian splat, area-normalised peak,
  │      σ floor 0.8 mm, confidence fade, coast ring, edge glow; ack {seq, shown, latencyUs, I_chain} back over the link
  └─► MagView / UI service (RP2350 core 0): orbit, follow, POV camera; menu with LOCAL and REMOTE_CH32 items; log tee
```

On the single-chip test bed the link is a loopback and the renderer reads the tracker's state directly, as today.

### 9.2 State

- **Per sensor**: `baseline[3]` (mT, slow IIR), `sigmaLive[3]` (from the baseline residual), `gain[3]` or `G[3][3]` and `offset[3]` from `magcal`, `K[3][n_cur]` µT/A from `magcal ledcal` (V6), `lastSetCount`, `badFrames`, `skipped`, `samplesThisFrame`.
- **Frame**: `frameUs` (mean read time), `readsInFrame`, the frame's `I[n_cur]` (chain and rail currents) as published by the render code.
- **Track** (`MagTrack`, as in the tree): three `MagTrackAxis {p, v, p00, p01, p11}`; `shaft` (unit) and its two 1€ axes; `state ∈ {NONE, ROUGH, COASTING, TRACKING}`; `droppedRun`, `droppedAt[3]`, `trackAge`, `coastFrames`, `speedAtLoss`, `speedSmoothed` (0.3 s), `moving`; three `OneEuroAxis` for the cursor.
- **Cursor**: `cursor`, `rawCursor`, `cursorSigmaMm`, `reachMm`, mode, `surfaceZ` (fit frame; = `magLocator.boardZ`, the one define), `tipOffsetMm`.
- **Link** (`ProbeLink`): the outgoing `ProbeCursorMsg` (§9.5), `seq`, the last ack `{seq, shown, latencyUs, layoutId, brightness}`, `rpAlive` (an ack within 500 ms).
- **Renderer** (RP2350 core 1): the newest complete `ProbeCursorMsg` (seq, CRC, COBS-framed), `t_arrive`, `baseFrame[480]`, the derived `C`, `C⁻¹`, `peak`, and the ack returned over `NOE` after `IRQ_RP2CH`. On the test bed the same fields come from a `ProbeCursorState` written by the tracker and read by core 1 like `LogoRing`.
- **Fallback** (`MagEkf`, only if it wins on recordings): x = [p, v, m] (9), P 9 × 9, sequential scalar updates over the 3N readings.

**The matrices**, so this section stands alone. Per axis, T = dt measured from the frame stamps (10–14.5 ms today, §2.4), setting (A) of §5.3 as the bench default with (B) noted:

```
F = [[1, T], [0, 1]]                       x⁻ = F x
Q = σa² [[T⁴/4, T³/2], [T³/2, T²]]          (A) σa = 1000 mm/s² dwelling, 2500 moving  |  (B) σa = 10000 single
moving ← |v̂|₀.₃ₛ > 50 mm/s && |v̂|² > 9 ΣP₁₁ ;  dwelling ← |v̂|₀.₃ₛ < 30 mm/s          (A only)
H = [1, 0]                                  R = max(σ_fit, 0.5 mm)²,  the floor = σ̂_R from Δ²x at rest
S = P₀₀ + R                                 d² = Σ_axes ν²/S
(A) w = 1 if d² ≤ 11.3 ; 11.3/d² if d² ≤ 90.8 (R ← R/w, S ← P₀₀ + R/w) ; drop if d² > 90.8
(B) per axis: w = 1 if d² ≤ 11.3 ; 3.37/√d² if d² ≤ 25 ; drop if d² > 25
K = [P₀₀, P₀₁] / S                          drop unconditionally when |ν| > 2500 mm/s × T (25 mm at 10 ms); no acceleration or reversal rule
P⁺ = [[(1−K₀)P₀₀, (1−K₀)P₀₁], [·, P₁₁ − K₁P₀₁]]
coast: v ← v·exp(−T/0.15 s); track 300 ms; cursor drawn while ring = max(2√P₀₀, a_hand t²/2) ≤ 5 mm  (a_hand 300 dwelling / 2500 moving)
LED covariance:  C = A Σ_board Aᵀ + 0.8² I,  A = [[2.54 ax, 2.54 ay], [cx, cy]] from RowGrid's affine map
```

### 9.3 Parameters: the tree today against what the research says

| Parameter | In `MagTracker.h` now | Research value | What decides it |
|---|---|---|---|
| process noise σ_a | 3000 mm/s² single level | (A) 1000 dwelling / 2500 moving, switched at 50 mm/s up / 30 down on a 0.3 s-smoothed speed, only if \|v̂\|² > 9 ΣP₁₁; or (B) 10000 single. Not 300/2500 on the filtered speed: it deadlocks (§5.2) | hop-onset simulation (0/20 rejected at 1000, 19/20 at 300); the `H` procedure below: at-rest \|v̂\| 95th percentile < 30 mm/s; rejections during a deliberately fast sweep decide between (A) and (B) |
| gate | 4σ per axis (`MAGTRACK_GATE`) | χ² 3 DOF: accept d² ≤ 11.3, Huber-weight to 90.8 (A) or per-axis to 25 (B), drop beyond | `H`: at-rest rejection 0–1 %, mean d² 0.3–1 (≈ 3 means R is exactly right, ≫ 3 the floor is too low) |
| σ floor for gating | 0.3 mm | 0.5 mm, measured as σ̂_R = std(Δ²x)/√6 at rest (expected 0.4–0.9): the array's systematic error, not the hand (the bench's 0.9 mm is drift the model tracks) | `H` with smoothing off; raise only if mean d² at rest > 3 |
| reinit agreement | 6 mm (`MAGTRACK_REINIT_AGREE_MM`) | 2500 mm/s × dt (human bound), 3 in a row | a fast sweep after a glitch must re-lock while moving |
| hard plausibility | max speed 3000 mm/s | keep 2500–3000 mm/s and apply it **unconditionally** (the only guard while trackAge < 3 and after a coast); **no acceleration or reversal rule**: a hand's a T² is 1–2 mm, under the 2.2 mm noise second difference | the bound is Casiez 2008's 2.4 m/s extreme |
| coast | 400 ms, v decays τ 0.15 s | keep τ 0.15 s; track 300 ms; cursor while the ring ≤ 5 mm (≈ 130 ms dwelling / 65 ms moving under (A)); ring = max(2√P₀₀, a_hand t²/2) | tap detector's 0.3 s tolerance |
| 1€ cursor | 1.0 Hz, β 0.1, d 1.0 Hz | 1.0 Hz, β 0.03–0.045, d 1.0 Hz | the paper's two-step procedure on the bench |
| 1€ shaft | τ 0.12 s exponential, 45° flip confirmation | 1€ at f_cmin 0.5 Hz (τ ≈ 0.32 s at rest), β 0.005 on tip speed | pointer jitter at 30 mm hover |
| reach cap | 40 mm | 5 mm for the hole cursor, 40 for the LCD pointer | |
| pointed blend | UNDER / POINTED | + AUTO: ramp h = 3–15 mm, fade when σ_pointer > 1.5 mm, off below 20° from horizontal; σ_c² = σ_tip² + w² h² σ_θ² | user test (flicker complaints) |
| surface plane z_s | `MAGLOC_BOARD_Z_MM` and `ROWCOUNT_TOUCH_Z_MM`, both 17.5 | one define, the calibrated tip rest height in the fit frame (z = 0 at the elements); V6 a priori ≈ 9.4 + h_tp mm with underside sensors, never 7.1 | `c`, or the one-tap "Measure rest height" |
| field smoothing | on (α 0.08 at weak signal), `alpha` passed to the tracker | off when the track is on; interim R_eff = N_s σ²; the constant becomes α = 1 − exp(−dt/0.12 s) meanwhile | replay the row-mode recordings both ways |
| row hysteresis | fixed 0.1 row | m = clamp(σ_smoothed² ln 4, 0.1, 0.45) rows | hold-still test gives σ_smoothed |
| presence | `MAGLOC_PRESENT_MT 0.04` on one smoothed axis (0.06 until 2026-09-18) | raw-frame χ²(24) with per-axis σ (Z at half): on at T > 43 for 3 frames, off below 35; ranges 79 / 97 mm (bench magnet, single / 4-frame block), 98 / 124 mm (V6 rod) | |
| baseline | 64 frames at boot; `MAGLOC_BASELINE_DRIFT 0.002` per frame | IIR α = 1 − exp(−dt/30 s) while absent; frozen while present; dipole check before re-zero after 60 s stuck; `MAG_BASELINE_FRAMES` a duration | |
| rough fixes | misfit ≤ 0.7, bar ≤ 60 mm, σ ≥ 3 mm | keep misfit ≤ 0.7 (a perfect fit on raw frames reaches it at ~59 mm with the bench magnet, ~75 mm with the V6 rod); bar ≤ 60 mm is never reached inside the misfit gate, so it is moot; add Algorithm 3 for 60–80 mm (bench magnet) / 70–100 mm (V6), σ_xy 25 mm, direction from the centroid, +15–25 mm with the 4-frame block | the moment: type it in (`k`) and re-derive the bands from `gap2_rerun.py` |
| board fields (V6) | | `LED_IQ_MA 0.35`, `LED_ICH_MA 5.0`, `K[8][3][n_cur]` µT/A from `magcal ledcal`, `RAIL_K[8][3][2]`, `BOARD_FIELD_GATE_UT 30`: widen the misfit gate by this for one frame when the predicted chain current changes by > 200 mA between frames | the §2.11 measurement on V5, then on the first V6 |
| fit budget | `LM_MAX_ITERATIONS 30`, `LM_DONE_MM 0.005`, full cold path from a failed warm fit | ≤ 12 iterations, `LM_DONE_MM 0.02` (the CRLB at 20 mm is 0.07–0.26 mm), a relative-cost stop, an evaluate budget of ~60 per frame, cold seeds one per frame with a best-so-far, no cold start inside a tracking frame | `fitUs` 99th percentile under 3 ms |
| frame | `MAG_FRAME_PERIOD_US 10000` as a floor; dt from the scheduler | phase-locked stamp; dt from the data; reads or LCD off the V5F | frame dt 10.0 ± 0.5 ms |
| LED | | σ_min 0.8 mm, σ₀ 1.3 mm, peak floor 0.12, confidence 0.25–1 over misfit 0.05–0.40, lead by measured age (clamp 40 ms, scaled by speed/50 mm/s), σ low-pass 5 Hz asymmetric, selection hysteresis 20 %, tail 0.35/0.12 at 1/2 pitches when tilt > 15°; ≥ 2 chains on V6, preferably 4 | host replay; `latencyUs` per chain split |
| UI | | debounce 20, hold 500, long 1500, repeat 400/80/40 ms; joystick dz 0.22, expo 0.5, digital 0.60/0.35; menu timeout 20 s; follow τ 150 ms, dead band 2 mm, slew 300 mm/s; POV dir τ 250 ms, pos 80 ms, dist 40 mm, zoom 3, el ≥ 15°, yaw freeze 8/12°, back-off max(tipOffset, 10 mm); preset spring 0.35 s | |
| sensor | 32×, ±40, tempco 00 (since 2026-09-18), T off, continuous, 400 kHz, median-of-3, CRC off | 32×, ±80, tempco 00, T off (also keeps the CRC byte in place), SET_COUNT gate instead of the median, LN, CRC off until §2.3 tests A/B pass then 3-byte mode, triggered general-call sampling at 1 MHz on V6 | §2.10 bench tests |

**The `H` procedure** (hold-still, smoothing off): 5 s hovering at 12–16 mm, 5 s with the tip in a hole, with ν, S, d², w, accepted, |v̂|, √ΣP₁₁, `moving`, `dd = x[k] − 2x[k−1] + x[k−2]`, `fitUs`, iterations and evaluate count per frame in the `d` stream. Report rejected %, mean d², the 95th percentile of |v̂|, and std(dd)/√6 per axis. Then 30 s of ordinary wiring for the 99th percentile of |dd| (the only defensible second-difference threshold, and a noise gate) and one deliberately fast sweep for the rejection count that decides (A) against (B).

### 9.4 Build order

Measure first, then the floor, because every estimator above bottoms out on it; then the track that exists, retuned with evidence; then what the user sees; then the V6 split.

1. **Measure the loop.** Add `micros()`, frame dt, reads-in-frame, LM iterations, `evaluate` count and `fitUs` to `printFixCsv`, and log `jOS.printStats()` (last/max/avg/overruns per service, the `onServices` command) before and after each change. Targets: frame dt 10.0 ± 0.5 ms; `fitUs` 99th percentile under 3 ms; no service `max_us` above 3 ms except a rate-limited cold start. Then the phase-locked frame stamp, the data-stamped dt, and the fit budget (`LM_MAX_ITERATIONS` 12, `LM_DONE_MM` 0.02, a relative-cost stop, ~60 evaluates per frame, one lattice slab or one seed per frame on a cold start, never a cold start inside a tracking frame); the LCD band push off the V5F (V3F from shared SRAM, or a start-and-poll DMA in the core's SPI) as soon as it is the largest hog left.
2. **Sensor honesty.** SET_COUNT / RESULT_STATUS / DIAG gate in `sampleSensors`, duplicates dropped, a per-axis range check, `MAG_TEMPCO 00`, ±80 mT, LN; then remove the median-of-3. Measure: distinct SET_COUNTs per frame, no-magnet σ before and after. The two per-frame locator constants become time constants (§2.10 item 11).
3. **Calibrate the floor.** `magcal` with three gains per sensor (then 3 × 3 if the residual stays structured), on a recording with a bare magnet held in plastic at 15–25 mm, the array moved away from the dev board's RJ45 and USB shells. Target: the 0.05 mT repeatable error down toward the 0.011 mT noise; misfit on held-out frames is the score. Type the measured moment (`k`) and re-derive the §4.5 bands from `gap2_rerun.py`.
4. **Live baseline and presence.** Raw-frame χ²(24) presence with per-axis σ, hysteresis and the three-frame rule replacing `MAGLOC_PRESENT_MT`; baseline as a slow IIR (τ 30 s) gated by it; the existing dipole-pollution check before any automatic re-zero; live σ stored with the baseline.
5. **Per-axis weights and the prediction seed in `MagFit`**: 1/σ² per axis, and the track's x̂⁻ as the warm start on every frame including after a miss; at most one leave-one-out refit per frame, on the worst-residual sensor, when the misfit is high.
6. **Retune `MagTracker` from recordings.** Add the `H` columns to the `d` stream; then setting (A): Q 1000/2500 with the significance-tested speed switch, the Huber-weighted χ² gate to 8×, the 0.5 mm floor checked against σ̂_R, the unconditional 2500 mm/s × dt bound, the agreeing-reject reinit, the 300 ms track coast with the ring from max(2√P₀₀, a_hand t²/2); no acceleration or reversal rule. Run `H` before and after (rejected %, mean d², 95th-percentile |v̂|), count rejections during a deliberately fast sweep, and move to (B) if it drops frames there. Switch the field smoothing off with the track on and compare on the ruler and row-mode recordings (with R_eff inflation as the interim). Keep `enabled = false` as the A/B switch.
7. **Cursor and rows.** AUTO blend with the 3–15 mm ramp, the σ_pointer fade and σ_c² = σ_tip² + w² h² σ_θ²; the 5 mm hole reach cap beside the 40 mm LCD cap; σ-scaled row hysteresis in `RowCounter`; the shaft 1€ on tip speed; one define for z_s (`MAGLOC_BOARD_Z_MM`, the calibrated tip rest height in the fit frame) with the V6 a priori (shell top 9.43 mm above the underside elements: 7.1 + 1.6 + 0.73, plus h_tp) documented next to it as a sanity check only; the "Measure rest height" one-tap action.
8. **LED renderer**, V5 table first since that board exists: the coordinate table, the affine covariance, the area-normalised splat, gamma with the low-end compensation, host ASCII replay for tuning, then the cursor plane in JumperlOS over `baseFrame` (never `requestLedShow`); the frame's `I_chain` published from `show()`; V6 table once the board file confirms the column geometry (the 2026-09-11 MainBoard has twelve hole rows with no channel and eight outer rows, 600 LEDs, §7.1; which outer rows are rails needs the schematic).
9. **`ProbeLink`**: message structs, COBS, CRC-16, the loopback back end on the test bed; the FSMC back end with the A0 = 1 channel, DMA-only writes, the NWAIT pull, the alive gate; the registry announce and `SET key value` for remote menu items; bench-verify the PIO demux and measure `latencyUs` per chain split before fixing the LED chain count in the board file (§9.5).
10. **UI**: `ButtonTracker` and joystick layer, the menu registry beside `consoleAddCommand` with LOCAL and REMOTE_CH32 items, the tee log, then the camera: orbit/pan/zoom, follow, POV with the 10 mm minimum back-off and near-plane clipping in `line3d` (required), preset springs.
11. **Board fields for V6** (§2.11): the V5 breakout measurement over the LED trunks and the VBUS run before the V6 routing is fixed; the layout rules into the board file (no trunk within 10 mm of a sensor, stacked returns, feed from both ends, switchers ≥ 25 mm away and shielded, ≥ 2 LED chains); `magcal ledcal` and the per-frame K·I subtraction; the TPS65131 beat test on its EVM or the first V6.
12. **Triggered sampling for V6**: standby mode, general-call trigger from a timer every 2.5 ms, shared INT ready pulse, 1 MHz bus or two peripherals; bench-verify with eight sensors and CRC off first, then the §2.3 CRC tests A and B, and 3-byte-mode CRC if they pass.
13. **`MagEkf`**, only if step 6 leaves the overhang and weak-signal segments worse than the per-frame fit: share the field and Jacobian code with `MagFit`, run both on the host on the same recordings, switch on evidence.
14. **Revisit the closed form** for a V6 array at ≤ 20 mm pitch after steps 3 and 12: the seed benchmark says 4–5× cheaper cold starts from 15 mm up on a grid that dense once the floor is calibrated (with the 0.05 mT floor it is a direction, not a seed); on the bench geometry the lattice search stays.

### 9.5 The V6 data path: tracker on the CH32H417, everything the user sees on the RP2350

The first draft described `ProbeCursorState` as "written by the tracker and read by core 1", one chip, two cores. On V6 the tracker and the LEDs are on different chips: rev B of the design notes (`notes/README_v6ch32H417.md`, *verified*) makes the link "the CH32's FSMC external-memory bus driven straight into RP2350 PIO pins … DMA-fed PIO FIFOs on the RP2350, hardware backpressure via NWAIT", 8 data bits, `FMC_NE1/NOE/NWE`, `FMC_A0 (reg vs FIFO)`, `FMC_NWAIT`, doorbells `IRQ_CH2RP` and `IRQ_RP2CH`; from the CH32H417 reference manual as read in chat 07 (*verified there, secondary here*) ~40 ns per byte with NWAIT, "~25 MB/s at 8-bit", bank 1 Device memory with no fences needed. `wireless-probe-sensing.md` §10 lists "which chip owns probe sensing" as open; this section closes it by recommendation. The test bed already draws the boundary: `ProbeLedService::printCursorLine` streams a `cursor,…` line (state, along, across, two σ, confidence, height, under-point) at 25 Hz over a UART and `ports/jumperlos/ProbeCursor` renders it on a V5 with the same `ProbeLeds` code. The split is right; it needs a binary form, a rate, a channel and an ownership rule, and its paint path (`requestLedShow(-1)`, a full `showNets` compose per line) must not become the V6 path.

**1. Ownership.** The CH32 owns array, baseline, fit, `MagTracker`, cursor modes, `RowGrid` and `RowCounter`: everything up to a cursor in breadboard terms with its uncertainty and the selected hole with hysteresis applied. The RP2350 owns the LED frame, hole → net semantics (`brightenNet`), the LCD, the inputs and the menu. The message carries breadboard coordinates and a hole, never array geometry. The I²C traffic and the 100 Hz fit belong on the chip with nothing else at that rate; RP2350 core 1 is the LED and crossbar core, core 0 runs JumperlOS and MicroPython; the V5 port has proven the boundary. It is the touch-controller pattern: coordinates and confidence at a fixed rate, the host renders.

**2. The message, at every fit.** A fixed 34-byte little-endian struct, no padding, once per fit (100 Hz): 3.4 KB/s against ~25 MB/s. The RP2350 keeps the newest complete one and picks it up at its own cadence.

```c
struct ProbeCursorMsg {                  // CH32 -> RP2350, channel FMC_A0 = 1, every fit
    uint8_t  magic, ver;                 // 0xC5, 1
    uint16_t seq;
    uint16_t age_us;                     // sample midpoint -> send. An AGE, never a timestamp: no shared clock
    uint8_t  state;                      // NONE / ROUGH / COASTING / TRACKING
    uint8_t  flags;                      // bit0 haveUnder, bit1 tail valid, bit2 click, bit3 layout V5
    int16_t  along_q8, across_q8;        // rows x256, mm x256
    uint16_t sigAlong_q8, sigAcross_q8;  // 1-sigma
    int8_t   rho;                        // correlation x127 (C12 = rho sa sc)
    uint8_t  confidence;                 // x255
    int16_t  vAlong_q4, vAcross_q4;      // rows/s x16, mm/s x16, the CV Kalman velocity
    uint16_t height_q4;                  // mm x16
    int8_t   tailA, tailC;               // unit tail direction x127
    int16_t  underAlong_q8, underAcross_q8;
    uint8_t  selRow, selHole;            // 1..60 / 1..6 (7, 8 rails), 0 none; hysteresis applied on the CH32
    uint16_t crc16;                      // CCITT
};
```

Framing: the link is a byte FIFO with no message boundary, and the RP2350 reboots more often than the CH32 (every UF2 flash; the CH32 can force its bootloader, design notes §1), so the stream must resynchronise: COBS with a 0x00 delimiter (Cheshire and Baker, IEEE/ACM ToN 1999, *recalled*), one overhead byte per 254 and a `memchr` to the next frame after an error. `seq` detects loss, `crc16` a torn frame, which is dropped. Acknowledgement, RP2350 → CH32, read through `NOE` only after `IRQ_RP2CH` (never poll an empty TX FIFO under NWAIT): `{seq echoed, shown: 0 shown / 1 menu / 2 app owns strip / 3 off / 4 stale, latencyUs = t_show − t_arrive, layoutId, brightness, I_chain[]}` once per LED frame. Telemetry, a gate (stop streaming when nobody draws) and the current for §2.11; the lead is not derived from it.

**3. A channel of its own, and NWAIT is a liability.** A 1 MB capture at ~25 MB/s holds the FIFO for ~40 ms, four cursor frames; a cursor queued behind it is head-of-line blocked. The README reserves `FMC_A0 (reg vs FIFO)`: writes with A0 = 1 are the control channel. On the PIO side two state machines each wait for `NE1 & NWE`, each tests A0 with `jmp pin` (*recalled*) and one discards what the other keeps, so each channel has its own RX FIFO and DMA ring; or A0 is sampled as a ninth `in` bit and demultiplexed in software. Either is a bench check, not a re-spin (RP2350 PIO's system-writable RX-FIFO registers, `FJOIN_RX_GET`, *verified*, hold 16 bytes: too small). Chat 07 sells NWAIT as the RP2350 stalling the CH32 "while it services LEDs or a MicroPython GC pause"; for a 100 Hz loop that is a hazard: a CPU store with NWAIT held waits indefinitely, and the RP2350 is routinely not running the PIO (`pauseCore2ForFlash`, a UF2 flash, the forced bootloader). Rules: link writes go through M2M DMA so a stall parks a DMA channel, not the tracker; a pull resistor on `FMC_NWAIT` to the not-waiting polarity; the CH32 gates on an RP-alive heartbeat (an ack within 500 ms) and drops cursor frames rather than waits.

**4. Ownership on the RP2350: a cursor plane, not a re-render.** From `CoreMailbox.h` and `Commands.cpp` (*verified*): core 1 composes `showNets → showLEDmeasurements → showAllRowAnimations → renderGraphicOverlays` under `LED_NETS`; `LED_MENU` is menu text on the breadboard; `LED_GFX` means an app owns the strip (`ledGraphicsOwned()`); `show()` drops a frame when the DMA is busy. The rule: the cursor never requests a mode or takes `LED_GFX`, is suppressed under `LED_MENU`, `LED_GFX` and the user toggle (saying why in the ack), and never posts `requestLedShow(-1)` as the V5 port does. At the end of every full compose core 1 copies the frame to `baseFrame[480]` (1.9 KB); each cursor tick copies base → out for the σ-box pixels, adds the splat (§7.4 loop) and shows; JumperlOS's own layers are untouched. That is a hardware cursor plane: Linux DRM keeps `DRM_PLANE_TYPE_CURSOR` apart from `DRM_PLANE_TYPE_PRIMARY` so the pointer moves at input rate without re-rendering the primary (`include/drm/drm_plane.h`, *verified*). A click (`flags` bit 2) goes to `brightenNet` on the RP2350, which maps `(selRow, selHole)` to a net.

**5. Latency and the lead** are in §7.3: the link is ~1.5 µs per message, the phase wait for a free WS2812 DMA is what the split adds, the mean chain is 24–31 / 15–19 / 11–13 ms on 1 / 2 / 4 chains, and core 1 extrapolates by the measured age (LaValle's Method 2), clamped at 40 ms and scaled down at low speed.

**6. The alternative, render on the CH32 and ship LEDs**, is not bandwidth-limited (480 × 3 B × 100 Hz = 144 KB/s, 0.6 % of the link) but loses compositing (only the RP2350 knows net colours, brightness policy and who owns the strip) and show-time prediction. A sparse patch (origin, ≤ 8 × 8 levels + colour, ~200 B) is the fallback if core 1 is short; `probeLedRender` costs ~0.2 ms on the M33 (480 `expf`), the §7.4 σ-box loop less, so the state message is preferred.

**7. The LCD and the menu registry.** The RP2350 drives the panel and owns the registry (§8.4): CH32 items are REMOTE, announced at link-up, edited by `SET key value`, their action replies landing in the log tee. That is rpmsg's shape, "a virtio-based messaging bus that allows kernel drivers to communicate with remote processors" (kernel docs, *verified*), reduced to three channels: cursor, console, capture. `MagView` is plain C++ on a `GFXcanvas16`: port it and render from a 64-byte fix message (magnet, shaft, tip, pointer, σ, misfit, state) at 100 Hz. Fallback if core 0 is short: the CH32 renders and pushes dirty 15-row bands as RFB-style rectangles ("a sequence of rectangles of pixel data that the client should put into its framebuffer", RFC 6143 §7.6.1, *verified*), 115 KB × 18 fps ≈ 2 MB/s, with camera input then crossing the link the other way. Testolomew now: a `ProbeLink` module with loopback and FSMC back ends, so the split is tested before the board exists.

## 10. Sources

Local copies of PDFs, text dumps and the scripts named in §4–§5 are in `~/.claude/jobs/bcb6e98c/tmp/research/` (`tmag5273.pdf`/`.txt`, `ds_tmag5173-q1.txt`, `oneeuro.pdf`/`.txt`, `oculus_bp.pdf`/`.txt`, `ch32h417ds.pdf`/`.txt`, `tps65131.pdf`/`.txt`, `xl1010.pdf`/`.txt`, `anp047.pdf`, `lavalle2014.pdf`, `ang2004.pdf`, `riviere1998.pdf`, `tremor_duval2005_abstract.txt`, `compassmot.cpp`, `compass_adv.html`, `drm_plane.h`, `rpmsg.html`, `rfc6143.txt`; the twenty-four `e2e_<n>.md` threads; the simulations `farfield_sim.py`, `crlb.py`, `planefit_gate.py`, `gap2_rerun.py`, `gap1_cv_analysis.py`, `gap1_rest_sim.py`, `gap1_move_sim.py`, `gap7_gate.py`, `gap7_onset.py`, `gap7_minjerk.py` with their `*_out.txt`; the host benchmarks `seed_bench.cpp`, `seed_bench2.cpp`, `coldcost_bench.cpp`, `MagFit_tol.cpp`, `holdstill.cpp` (against `Testolomew/tools/recordings/2026-09-17-ruler-test.txt` and `2026-09-17-all-orientations.txt`); and the seven section notes plus `gap-1.md` … `gap-8.md` under `sections/`). The section notes themselves still carry the first draft's numbers where this document corrects them (the 7.1 mm surface plane, the 9,800 magnet, the 300/2500 Q, the field-notes "[all verified in-browser]" header); this document is the authority.

**TI documents** (`https://www.ti.com/lit/pdf/<id>` or `ti.com/lit/ds/symlink/<part>.pdf`)
- TMAG5273 datasheet SLYS045C, June 2021 rev. April 2026. https://www.ti.com/lit/ds/symlink/tmag5273.pdf
- SLYA051B, Linear Hall-Effect Sensor Array Design (Bryson, 2020 rev. 2024). https://www.ti.com/lit/pdf/slya051
- SLYA059B, Magnet Selection for Linear Position Applications. https://www.ti.com/lit/pdf/slya059b
- TIDUF78 / TIDA-060045, Accurate Low Latency Linear Position Sense Reference Design With Quad 3D Hall-Sensors. https://www.ti.com/lit/pdf/tiduf78
- SBAA540A, Multi-mover Position Sensing; SLYA070A, Sensor Array Fan-out (Staebler, Bryson). https://www.ti.com/lit/pdf/sbaa540, https://www.ti.com/lit/pdf/slya070a
- SBAA539A, Achieving Highest System Angle Sensing Accuracy (Bryson, 2023). https://www.ti.com/lit/pdf/sbaa539
- SBAA463, SBAA513B, SBAA519A, SLIA086A, SLYU058D, SLYU064A, SLYA080, SLYA074, TIDUC07 (introductory or single-sensor).
- TMAG5170, TMAG5173-Q1 (lifetime drift, `ds_tmag5173-q1.txt`), TMAG3001 datasheets. Tools: TIMSS https://webench.ti.com/timss/, SBAR012 https://www.ti.com/lit/zip/sbar012.
- TPS65130/TPS65131, SLVS493E (2004, rev. 2022): §5 f_S, §8.2.2.2 Eq 3–6 and Table 8-3, §10 layout. https://www.ti.com/lit/ds/symlink/tps65131.pdf

**TI E2E threads** (`https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/<n>/`; `…/<n>/x` redirects to the canonical URL). *Verified and saved verbatim* (24): 1200004 (CRC errata, Bryson; Pilhofer's single-device frames), 1121557 (4-byte minimum, Rosenberger; Wiget's every-4-bytes prose), 1232741 (CRC on EVM, Simmons; the T-on 5-byte case), 1357408 (6-byte 3-byte-mode CRC frames, Cacula), 1558871 / 1592349 (previous address folded into the CRC, Melnikoff), 1259072 (result atomicity, saturation, Baker), 1171980 (W&S interrupts stuck; 15 h zero-mismatch run), 1618505 (latched INT, Rosenberger), 1519712 (frozen SET_COUNT), 1357951 (no software reset), 1498216 (OSC_ER), 1394437 (bus pins high with VCC off, Bryson), 1223356 (pull-ups on a 16-sensor array), 1624039 (pull-up value), 1433746 (power cycling), 1661100 (no EEPROM), 1309963 (element position, Rosenberger), 1467768 (gain formula, Bryson), 1613640, 1329900 (noise, low result bits), 1590481 (lifetime drift), 1591522 (Cpk), 1402689 (error equations). *Recalled*: 1567765 (the 1000 h / 125 °C condition), 1371662, 1481520 (the 0.25° → 0.0625° PCN), 1346794, 1325695, 1366574, 1226788, 1106308, 1556835, 1348288, 1347326; nothing decision-driving hangs on them.

**Drivers and code**: SparkFun_TMAG5273_Arduino_Library (issues #5, #6, #9); Adafruit_TMAG5273; Linux `drivers/iio/magnetometer/tmag5273.c`; Zephyr `drivers/sensor/ti/tmag5273` and `dts/bindings/sensor/ti,tmag5273.yaml`; github.com/TuYuxiao/TMAG5273; forum.arduino.cc thread 1455979. Other datasheets: Allegro ALS31313, Melexis MLX90393 Rev 012, MEMSIC MMC5983MA Rev A; LIS3MDL and AK09940A from search snippets only.

**Magnet tracking with sensor arrays**
- Schlageter, Besse, Popovic, Kucera, Sens. Actuators A 92:37–42, 2001. https://www.sciencedirect.com/science/article/abs/pii/S0924424701005374
- Hu, Meng, Mandal, IEEE Trans. Magn. 43(12), 2007, doi:10.1109/TMAG.2007.907581
- Hu, Li, Song, Yang, Zhang, Meng, IEEE Sensors J. 10(5):903–913, 2010. https://ieeexplore.ieee.org/document/5443691/
- Hu et al., Med. Devices: Evidence and Research, 2015. https://www.dovepress.com/article/download/21235
- Hu, Meng et al., IEEE EMBS 2005 (PubMed 17281923); PMC4399597.
- Song, Li, Qiao, Hu, Ren, Yu, IEEE Trans. Magn. 2014, doi:10.1109/TMAG.2014.2315592
- Farajidavar, Block, Ghovanloo, IEEE EMBC 2012. https://pmc.ncbi.nlm.nih.gov/articles/PMC4445074/
- Son, Yim, Sitti, IEEE/ASME Trans. Mechatronics 21(2), 2016. https://ieeexplore.ieee.org/document/7293677/
- Micromachines 17(3):327, 2026. https://pmc.ncbi.nlm.nih.gov/articles/PMC13028852/
- Sensors 20(20):5728, 2020. https://pmc.ncbi.nlm.nih.gov/articles/PMC7601872/
- Li, arXiv:2201.02372, 2022. https://arxiv.org/pdf/2201.02372
- Cichon, Psiuk, Brauer, Töpfer, IEEE Sensors J. 19(7):2509–2516, 2019. https://ieeexplore.ieee.org/document/8603829/
- Fischer, Kriechbaum, Berwanger, Mathis-Ullrich, IEEE Sensors Letters 2023, doi:10.1109/LSENS.2023.3250971
- Taddese, Slawinski, Pirotta, De Momi, Obstein, Valdastri, IJRR 37(8):890–911, 2018. https://eprints.whiterose.ac.uk/130353/1/MAC_IJRR_2017.pdf
- Wahlström, Gustafsson, IEEE Trans. Signal Process. 62(3):545–556, 2014. https://liu.diva-portal.org/smash/get/diva2:606532/FULLTEXT01 ; thesis https://www.diva-portal.org/smash/get/diva2:606554/FULLTEXT01.pdf
- Birsan, IEEE Trans. Magn. 47:409–415, 2011. https://ieeexplore.ieee.org/document/5629441/ ; OCEANS 2005, doi:10.1109/OCEANS.2005.1639993
- Ge, Song, Wang, Meng, IEEE Sensors 2021. https://ieeexplore.ieee.org/document/9639620/
- Hou, Xu, Ho, Cai, Doğançay, Xu, ICASSP 2026, doi:10.1109/ICASSP55912.2026.11463962
- Raab, Blood, Steiner, Jones, IEEE Trans. AES 15:709–718, 1979. https://www.semanticscholar.org/paper/b1a672fc01064dcb0658050bf0ea5df2eaab4f49
- Kindratenko, Virtual Reality 2000, doi:10.1007/BF01409422 ; Franz et al., IEEE TMI 2014, doi:10.1109/TMI.2014.2321777
- Harrison, Hudson, Abracadabra, UIST 2009, doi:10.1145/1622176.1622199
- Chen, Lyons, White, Patel, uTrack, UIST 2013. https://kentlyons.net/pubs/utrack-uist13.pdf
- Hwang, Bianchi, Ahn, Wohn, MagPen, MobileHCI 2013, doi:10.1145/2493190.2493194
- Chen, Patel, Keller, Finexus, CHI 2016. https://ubicomplab.cs.washington.edu/pdfs/finexus.pdf
- Parizi, Whitmire, Patel, AuraRing, IMWUT 3(4), 2019. https://dl.acm.org/doi/10.1145/3369831
- MagSurface, arXiv:2105.00543. https://arxiv.org/pdf/2105.00543 ; µTouch, arXiv:2601.22864. https://arxiv.org/pdf/2601.22864
- Sensors 25(20):6444, 2025 (TinyML Hall array). https://pmc.ncbi.nlm.nih.gov/articles/PMC12568014/
- Kok, Schön, IEEE Sensors J. 2016, arXiv:1601.05257. https://arxiv.org/abs/1601.05257
- Iivanainen et al., 2022. https://pmc.ncbi.nlm.nih.gov/articles/PMC9024658/
- Petruska, Abbott, IEEE Trans. Magn. 49(2):811–819, 2013. https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=3c3755861c9e4cb78017c442f22edf8bf7854433
- Derby, Olbert, Am. J. Phys. 78(3):229–235, 2010. https://arxiv.org/abs/0909.3880
- Singer, IEEE Trans. AES 1970, doi:10.1109/TAES.1970.310128 ; Bryson, Henrikson, AIAA 1967, doi:10.2514/6.1967-541 ; Bell, Cathey, IEEE TAC 1993 (recalled); Rauch, Tung, Striebel 1965 (recalled).

**Closed-form and far-field**
- Nara, Suzuki, Ando, IEEE Trans. Magn. 42(10):3291–3293, 2006, doi:10.1109/TMAG.2006.879151
- Nara, Ito, J. Appl. Phys. 115:17E504, 2014, doi:10.1063/1.4861675 ; Higuchi, Nara, Ando, Int. J. Appl. Electromagn. Mech. 52:67–72, 2016 ; Nara, Watanabe, Ito, IEEE Trans. Magn. 48(11):4444–4447, 2012
- Wu, Luo, Pu, Wei, Chen, Zhang, arXiv:2606.01946, 2026. https://arxiv.org/pdf/2606.01946
- Wang et al., Sensors 16(12):2168, 2016. https://pmc.ncbi.nlm.nih.gov/articles/PMC5191147/
- Wang et al., Micromachines 13(10):1639, 2022. https://pmc.ncbi.nlm.nih.gov/articles/PMC9607652/
- You et al., Sci. Rep. 12:17985, 2022. https://pmc.ncbi.nlm.nih.gov/articles/PMC9605944/
- Sensors 24(19):6194, 2024 (PMC11435889); Sensors 24(7):2224, 2024 (PMC11014331); PMC13418459, 2026.
- Reid et al., Geophysics 55(1):80–91, 1990. https://www.reid-geophys.co.uk/wp-content/uploads/2017/11/Reid-et-al-1990.pdf
- Wiegert, Oeschger, OCEANS 2005; Wiegert et al., OCEANS 2007, 2008 (DTIC ADA502334). Sui et al., IEEE Trans. Magn. 48:4701, 2012; Sui, Leslie, Clark, IEEE Magn. Lett. 8, 2017; Clark, Explor. Geophys. 43:267, 2012; Beiki et al., Geophysics 77:J23, 2012; Yin et al., JMMM 499:166274, 2020; Xu, IEEE TIM 70, 2021; Tang et al., Remote Sens. 15:4959, 2023.
- Wynn, NSRDL rep. 3493, 1972; Frahm, NCSL, 1972; Wynn et al., IEEE Trans. Magn. 11:701–707, 1975.
- Ginzburg, Frumkis, Kaplan, Sens. Actuators A 102:67–75, 2002; Sheinker et al., IEEE Trans. Magn. 45:160–167, 2009; Chenevas-Paule et al., arXiv:2504.05212, 2025.

**Motion filtering, pointing, selection**
- Casiez, Roussel, Vogel, 1€ Filter, CHI 2012. https://gery.casiez.net/publications/CHI2012-casiez.pdf ; https://gery.casiez.net/1euro/
- Taranta, Koh, Williamson, Pfeil, Pittman, LaViola, Pitch Pipe, Graphics Interface 2019. https://www.cs.ucf.edu/~jjl/pubs/gi19.pdf
- Shao, Wang, Zhou, Tran, Krishnan, Nayar, N-euro Predictor, IMWUT 7(3), 2023. https://cave.cs.columbia.edu/Statics/publications/pdfs/Shao_Mob23.pdf
- Vogel, Balakrishnan, Distant Freehand Pointing and Clicking, UIST 2005. https://www.dgp.toronto.edu/~ravin/papers/uist2005_distantpointing.pdf
- Casiez, Vogel, Balakrishnan, Cockburn, The Impact of Control-Display Gain, HCI 23(3), 2008. https://www.dgp.toronto.edu/~ravin/papers/hci2008_cdgain.pdf
- Argelaguet, Andujar, A survey of 3D object selection techniques, Computers & Graphics 37(3), 2013. http://www.cs.ucf.edu/courses/cap6121/spr15/readings/3Dselection.pdf
- Kopper, Bowman, Silva, McMahan, IJHCS 68:603–615, 2010.
- Bi, Zhai, Bayesian Touch, UIST 2013. https://research.google.com/pubs/archive/41644.pdf
- Grossman, Balakrishnan, A Probabilistic Approach to Modeling Two-Dimensional Pointing, TOCHI 12(3), 2005. https://www.dgp.toronto.edu/~ravin/papers/tochi2005_probabilistic.pdf
- Grossman, Balakrishnan, The Bubble Cursor, CHI 2005. https://www.dgp.toronto.edu/~ravin/papers/chi2005_bubblecursor.pdf
- Blanch, Guiard, Beaudouin-Lafon, Semantic Pointing, CHI 2004. http://iihm.imag.fr/blanch/publications/chi2004/chi2004-sp.html
- Kabbash, Buxton, The Prince Technique, CHI 1995. https://www.billbuxton.com/prince.html
- Worden, Walker, Bharat, Hudson, Area Cursors and Sticky Icons, CHI 1997. https://sites.cc.gatech.edu/gvu/people/faculty/hudson/papers/older_adults/cursors_paper.html
- Lank, Cheng, Ruiz, Endpoint prediction using motion kinematics, CHI 2007. https://www.ruizlab.org/wp-content/uploads/2023/06/Endpoint-prediction-using-motion-kinematics.pdf
- Or, Selection-Induced Contraction of Innovation Statistics in Gated Kalman Filters, arXiv:2512.18508, 2026. https://arxiv.org/abs/2512.18508
- Wikipedia, Radar tracker https://en.wikipedia.org/wiki/Radar_tracker ; Alpha beta filter https://en.wikipedia.org/wiki/Alpha_beta_filter
- Kalata, The Tracking Index, IEEE TAES 20(2), 1984 (recalled). Bar-Shalom, Li, Kirubarajan, Estimation with Applications to Tracking and Navigation, Wiley 2001 (recalled). Blom, Bar-Shalom, IMM, IEEE TAC 33(8), 1988 (recalled). Flash, Hogan, J. Neurosci. 5(7), 1985 (recalled). Gandhi, Mili 2010 (recalled).
- MathWorks, Hampel Filter https://www.mathworks.com/help/dsp/ref/hampelfilter.html ; trackingIMM https://www.mathworks.com/help/fusion/ref/trackingimm.html
- Russell (IBM), US Patent 5,239,489, 1993. https://patents.google.com/patent/US5239489A/en
- Wacom Developer Support, Pen sample rate https://developer-support.wacom.com/hc/en-us/articles/9354474354583-Pen-sample-rate ; Apple Developer, coalesced touches and `UITouch.azimuthAngle(in:)`.
- Morrison, Newell, Human Movement Science 20, 2001. https://pubmed.ncbi.nlm.nih.gov/11750682/ ; Paulus, Remijn, Displays 2021 (gaze dwell).
- Stiles, Frequency and displacement amplitude relations for normal hand tremor, J. Appl. Physiol. 40(1):44–54, 1976. https://pubmed.ncbi.nlm.nih.gov/1248981/ (abstract *verified*)
- Ang, Active Tremor Compensation in Handheld Instrument for Microsurgery, CMU-RI-TR-04-28, 2004, §1.1 (quotes Hunter 1993, Singh 2002, Riviere 1997). https://www.ri.cmu.edu/pub_files/pub4/ang_wei_tech_2004_1/ang_wei_tech_2004_1.pdf
- Riviere, Rader, Thakor, Adaptive canceling of physiological tremor for improved precision in microsurgery, IEEE Trans. Biomed. Eng. 45(7), 1998. https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/riviere_cameron_1998_1/riviere_cameron_1998_1.pdf
- Gaveau, Papaxanthis, The temporal structure of vertical arm movements, PLoS ONE 6(7):e22045, 2011. https://pmc.ncbi.nlm.nih.gov/articles/PMC3134452/
- Duval, Jones, Vertical and horizontal components of physiological tremor, Exp. Brain Res. 163(2):261–266, 2005, PMID 15912372. https://doi.org/10.1007/s00221-005-2233-x (abstract *verified*)
- Gasser, Sroka, Jennen-Steinmetz, Residual variance and residual pattern in nonlinear regression, Biometrika 73:625, 1986 (recalled).

**LEDs and uncertainty display**
- Uncertain Pointer, CHI 2026, arXiv:2602.13433. https://arxiv.org/html/2602.13433v1
- Schwarz, Hudson, Mankoff, Wilson, UIST 2010; Schwarz, Hudson, UIST 2011.
- Tobii, Building for UX: connecting eye gaze to UI objects. https://www.tobii.com/resource-center/learn-articles/building-for-ux-connecting-eye-gaze-to-ui-objects
- Hassoumi, Peysakhovich, Hurter, J. Eye Movement Research 10(5), 2018. https://pmc.ncbi.nlm.nih.gov/articles/PMC7141080/
- Correll, Gleicher, Error Bars Considered Harmful, IEEE TVCG 20(12), 2014. https://graphics.cs.wisc.edu/Papers/2014/CG14/Preprint.pdf
- Correll, Moritz, Heer, Value-Suppressing Uncertainty Palettes, CHI 2018. https://dl.acm.org/doi/10.1145/3173574.3174216
- Hullman, Resnick, Adar, Hypothetical Outcome Plots, PLOS ONE 2015. https://idl.cs.washington.edu/files/2015-HOPs-PLOS.pdf
- US Patent 9,292,971 (area of uncertainty display); radartutorial.eu.
- Kriegsman, anti-aliased light bar, FastLED 2013. https://pastebin.com/g8Bxi6zW
- Mountain Lizard, Gamma for WS2812. https://mountainlizard.com/posts/gamma-ws2812/
- FastLED Temporal Dithering https://github.com/FastLED/FastLED/wiki/FastLED-Temporal-Dithering ; FastLED issue #4042.
- WS2812B datasheet. https://www.ledyilighting.com/wp-content/uploads/2025/02/WS2812B-datasheet.pdf
- Deber, Jota, Forlines, Wigdor, How Much Faster is Fast Enough?, CHI 2015. https://www.tactuallabs.com/papers/howMuchFasterIsFastEnoughCHI15.pdf ; Ng et al., UIST 2012. https://dl.acm.org/doi/10.1145/2380116.2380174 ; Jota, Ng, Dietz, Wigdor, CHI 2013.
- Wikipedia, CIELAB (L* formula).

**UI and camera**
- LVGL `lv_conf_template.h` https://raw.githubusercontent.com/lvgl/lvgl/master/lv_conf_template.h ; Menu widget https://lvgl.io/docs/open/widgets/menu
- u8g2 wiki: u8g2reference https://github.com/olikraus/u8g2/wiki/u8g2reference ; muimanual https://github.com/olikraus/u8g2/wiki/muimanual
- Ganssle, A Guide to Debouncing. http://www.ganssle.com/debouncing.htm ; http://www.ganssle.com/debouncing-pt2.htm
- Microsoft, SystemParametersInfoA https://learn.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-systemparametersinfoa ; Getting Started With XInput https://learn.microsoft.com/en-us/windows/win32/xinput/getting-started-with-xinput
- Sutphin, Doing Thumbstick Dead Zones Right. https://joshsutphin.com/gamedev/doing-thumbstick-dead-zones-right.html
- Betaflight `src/main/fc/rc.c`. https://raw.githubusercontent.com/betaflight/betaflight/master/src/main/fc/rc.c
- Juckett, Damped Springs, 2012. https://www.ryanjuckett.com/damped-springs/
- Unity, UnityCsReference `Runtime/Export/Math/Mathf.cs`. https://raw.githubusercontent.com/Unity-Technologies/UnityCsReference/master/Runtime/Export/Math/Mathf.cs
- Driscoll, Frame Rate Independent Damping Using Lerp, 2016. https://www.rorydriscoll.com/2016/03/07/frame-rate-independent-damping-using-lerp/
- Oculus, Best Practices, 310-30000-02, 2017. https://static.oculus.com/documentation/pdfs/intro-vr/latest/bp.pdf
- Marlin `Configuration_adv.h`, bugfix-2.1.x. https://raw.githubusercontent.com/MarlinFirmware/Marlin/bugfix-2.1.x/Marlin/Configuration_adv.h
- Haigh-Hutchinson, Real-Time Cameras, 2009 (recalled); Shoemake, SIGGRAPH 1985 (recalled).
- LaValle, Yershova, Katsev, Antonov, Head Tracking for the Oculus Rift, ICRA 2014, §VII. https://msl.cs.illinois.edu/~lavalle/papers/LavYerKatAnt14.pdf

**V6 board, data path and the board's own fields**
- `~/Desktop/V6pile/notes/README_v6ch32H417.md` (rev B link, pin table, PIO sketch); `claude-chats/` chat 07 (CH32H417RM V1.7: FMC timing, NWAIT rule, ~25 MB/s, Device-type bank 1); `~/Desktop/V6pile/notes/README_TPS65131.md`.
- WCH, CH32H417/H416/H415 Datasheet V1.8, §1.4.1–1.4.2. https://ch32-riscv-ug.github.io/CH32H417/datasheet_en/CH32H417DS0.PDF ; openwch/ch32h417 README; arduino-core-ch32h4 1.5.0 `docs/icache.md`, `docs/memory-baseline.md`, `docs/hazards.md`, `libraries/SPI/src/SPI.cpp`, `cores/ch32h4/startup_v5f.S`.
- Layouts: `~/Documents/GitHub/JumperlessV6/Untitled/Untitled.kicad_pcb` (V5r7, routed), `~/Documents/GitHub/JumperlessV6/MainBoard/MainBoard.kicad_pcb` (V6, placement only, 2026-09-11) and `MainBoard.kicad_sch`, `JustTheLEDs.kicad_sch`; `~/Desktop/V6pile/purchasing/` (Spring Clip6 material).
- XINGLIGHT, XL-1010RGBC-WS2812B datasheet (I_DD 0.35 mA, I_OUT 5 mA/ch, f_PWM 1.0 kHz). https://files.seeedstudio.com/wiki/xiao-rgb-matrix/WS2812B-1010-DATASHEET.pdf
- Würth Elektronik, ANP047c, The Behavior of Electro-Magnetic Radiation of Power Inductors in Power Management. https://www.we-online.com/components/media/o109027v410%20ANP047c_The%20Behavior%20of%20Electro-Magnetic%20Radiation%20of%20Power%20Inductors%20in%20Power%20Management.pdf ; Coilcraft, Fundamentals of Electromagnetic Compliance. https://www.coilcraft.com/en-us/resources/application-notes/fundamentals-of-electromagnetic-compliance/
- Ott, Electromagnetic Compatibility Engineering; Johnson, Graham, High-Speed Digital Design (return-current crossover; recalled).
- ArduPilot, `ArduCopter/compassmot.cpp`, https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/compassmot.cpp ; Advanced Compass Setup, https://ardupilot.org/copter/docs/common-compass-setup-advanced.html
- Zhang et al., An Aeromagnetic Compensation Method for Suppressing the Magnetic Interference Generated by Electric Current with Vector Magnetometer, Sensors 22(16):6151, 2022, doi:10.3390/s22166151. https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9416573/
- Linux `include/drm/drm_plane.h` (`enum drm_plane_type`); rpmsg framework, https://docs.kernel.org/staging/rpmsg.html ; RFC 6143 §7.6.1, https://www.rfc-editor.org/rfc/rfc6143.txt
- RP2350 PIO RX-FIFO registers: https://github.com/raspberrypi/pico-sdk/issues/2261 ; RP2350 datasheet §11.1.1.
- Cheshire, Baker, Consistent Overhead Byte Stuffing, IEEE/ACM ToN 7(2), 1999 (recalled).

**Our own**: `Testolomew/docs/wireless-probe-sensing.md`; `src/magarray/{TMAG5273,MagArray}.{h,cpp}`, `MagArrayConfig.h`; `src/magfit/{MagFit,MagLocator,MagTracker}.{h,cpp}`; `src/rowcount/RowGrid.h`, `RowCounter.{h,cpp}`; `src/display/{MagView,ST7789}.cpp`; `src/jos/JumperlOS.cpp`; `src/ui/Ui.cpp`; `src/probeled/ProbeLedService.cpp`; `ports/jumperlos/ProbeCursor/*`; `tools/recordings/2026-09-17-*.txt`; `attic/README.md`, `attic/far-field-gradient/`; JumperlOS `src/LEDs.{h,cpp}`, `Graphics.cpp`, `main.cpp`, `Apps.cpp`, `Commands.cpp`, `coredination/CoreMailbox.h`, `eyecandy/Highlighting.h`, `eyecandy/Colors.h`, `eyecandy/GraphicOverlays.h`, `Jerial.{h,cpp}`, `oled.cpp`, `Menus.cpp`, `RotaryEncoder.{h,cpp}`, `remembering/menuTree.h`.

