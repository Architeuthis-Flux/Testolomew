// SPDX-License-Identifier: MIT
#ifndef BOARDPINS_H
#define BOARDPINS_H
// ---------------------------------------------------------------------------
// Every pin Testolomew uses, in one place. Modules take their pins from here
// and nowhere else, so rewiring the bench (or moving to a real V6 board) is an
// edit to this file only.
//
// The board is wuxx's nanoCH32H417 (CH32H417QEU6). Onboard wiring below is
// read off its schematic, hardware/nanoCH32H417-v1.0.pdf.
//
// I/O voltage: on this chip only PA5-PA7 and PE2-PE6 are on the fixed 3.3 V
// rail. Almost every other pin - including every pin the magnetometer array
// is on except PA8 - belongs to the "VIO18" domain, a rail the chip makes
// itself and that comes up at 1.2 V. The Arduino core raises it to 3.3 V
// during boot, which is the only reason a 3.3 V sensor can be powered from a
// GPIO here. Under any other framework (WCH's SDK, ch32fun) that has to be
// done by hand: PWR->CTLR bits [12:10] = 3. boardVioIs3V3() checks it.
// ---------------------------------------------------------------------------
#include <Arduino.h>

// ---- onboard -------------------------------------------------------------
#define PIN_LED_BLUE PC3  // D1, active low (LED to VDDIO through 1k)
#define PIN_LED_GREEN PC2 // D2, active low

// J8, the 12-pin FPC "SPI-LCD" connector. Backlight is wired straight to
// VDDIO (always on); there is no MISO.
#define PIN_LCD_SCK PB13 // SPI2
#define PIN_LCD_MOSI PB15
#define PIN_LCD_CS PB12
#define PIN_LCD_DC PD9
#define PIN_LCD_RST PD8

// The console is Serial = USART1 on PA9 (TX) / PA10 (RX), which the onboard
// WCH-LinkE bridges to USB through solder bridges SB4 / SB3 (see
// board_build.serial in platformio.ini). SWD is PB8 / PB9 through SB5 / SB6.

// ---- magnetometer array (header J5 and J9) --------------------------------
// 4x2 TMAG5273. One I2C bus, and each sensor's VCC on its own GPIO so they can
// be powered up one at a time to be given unique addresses (see MagArray.h).
// The bus needs real pull-up resistors to 3.3 V: this chip has no internal
// pull-ups in open-drain mode.
#define PIN_MAG_SCL PA8 // I2C3 (the silicon pairs PA8 with PC9)
#define PIN_MAG_SDA PC9

// Sensor VCC pins, sensor 0..7. Set A is on J5, set B on J9.
#define PIN_MAG_VCC_0 PD0
#define PIN_MAG_VCC_1 PC10 // moved from PC12 on 2026-09-17
#define PIN_MAG_VCC_2 PC11
// The wiring note listed PC12 twice; PA14 (the header pin between PC11 and the
// set's ground, PA15) was a guess, and a sensor answers on it (bus check,
// 2026-09-17), so it stays.
#define PIN_MAG_VCC_3 PA14
#define PIN_MAG_VCC_4 PD6 // moved from PD1 on 2026-09-17
#define PIN_MAG_VCC_5 PD2
#define PIN_MAG_VCC_6 PD3
#define PIN_MAG_VCC_7 PD4

// Spare pins on the same two headers. The bus check tries these as the supply
// of any sensor that does not answer on the pin listed above, to find a wire
// that is one header position off. Nothing else may be wired to them.
// (PD7 left this list when it became the LED strip's data pin: the bus check
// would otherwise drive a breadboard's data-in as a supply.)
#define PIN_MAG_VCC_CANDIDATES { PC12, PA13, PD1, PC8, PC7, PC6 }
#define PIN_MAG_VCC_CANDIDATE_NAMES { "PC12", "PA13", "PD1", "PC8", "PC7", "PC6" }

// Each set of four has its ground on a GPIO too; they are driven low and left.
#define PIN_MAG_GND_A PA15
#define PIN_MAG_GND_B PD5

// ---- controls (wired 2026-09-18) --------------------------------------------
// The navigation stick (ALPS RKJXM1015004, 8-way with a centre push, LCSC
// C97432), an analog joystick with a press, and two buttons (src/ui/Input.h).
// -1 = not fitted: the UI then takes the same events from keys typed on the
// console.
//
// The stick has four direction contacts A-D (a diagonal closes two
// neighbours) and a centre push that is a contact of its own: the ALPS
// drawing's circuit is  Com -o o- Push -> A/B/C/D, with two "Push" pads on
// the footprint. So the press is read on its own pin (PIN_NAV_PRESS); with
// no pin for it (-1) Input.cpp falls back to calling three or four
// direction contacts at once a press. The four direction defines are the
// contacts as wired; if a direction comes out turned, swap them here. The
// switches pull to ground and use the chip's pull-ups; the joystick is two
// potentiometers across 3.3 V into ADC pins (read at 12 bits, centre
// assumed at half scale - `j` prints the raw readings).
#define PIN_NAV_UP PD13
#define PIN_NAV_DOWN PD10
#define PIN_NAV_LEFT PE15
#define PIN_NAV_RIGHT PD15
#define PIN_NAV_PRESS PD14 // the stick's Push pad (wired 2026-09-18)
#define PIN_JOY_X PA3
#define PIN_JOY_Y PA4
// The stick as wired reads the other way along X (right gave a falling
// reading, 2026-09-19): 1 turns it round so + is right, as Input.h wants.
#define JOY_X_REVERSED 1
#define JOY_Y_REVERSED 0
#define PIN_JOY_PRESS PF2
// The push switch: closes to ground (1) or to 3.3 V (0), and whether the
// chip's pull-up is wanted on the pin. Read on the bench 2026-09-19
// (docs/bench/2026-09-19-nav-trace-and-inputs.txt): the pin is high at
// rest with the pull-up on, so it is an ordinary switch to ground.
#define JOY_PRESS_ACTIVE_LOW 1
#define JOY_PRESS_PULLUP 1
#define PIN_BTN_A PE14
#define PIN_BTN_B PD12

// ---- a V5's breadboard LEDs (wired 2026-09-18) --------------------------------
// The data-in of a V5's breadboard LEDs, driven straight from here
// (src/probeled/LedStrip.h) so the probe cursor shows on a real breadboard.
// The pin must be a MOSI: PD7 is SPI1's, whose SCK and MISO (PA5, PA6) are
// named too (the SCK pad is put back to an input once the SPI is set up,
// so nothing toggles on it). -1 = not wired. Grounds must be shared, and
// the data wire kept away from the sensor bus wires (it couples into SDA).
//
// The frame is 400 pixels: the 300 hole LEDs and then the 100 rail LEDs, as
// JumperlOS numbers them. On a V5 up to hardware revision 3 that is one
// chain and the rails light from this pin. A revision 4+ V5 has the rails
// at the start of a second strip with its own data-in (JumperlOS's "top"
// strip, LEDs.cpp) - wire that to PIN_LED_STRIP_TOP, another MOSI: PC12 or
// PA13 are SPI3's (AF6 / AF1) and free on the J5/J9 headers; SPI3's SCK and
// MISO then have to be PB3 and PB4, since its other choices (PC10, PA14,
// PC11, PC9) are sensor supplies and the I2C data line. -1 = no second
// strip (the rails still go out on the first chain, which is harmless).
#define PIN_LED_STRIP PD7
#define PIN_LED_STRIP_SCK PA5
#define PIN_LED_STRIP_MISO PA6
#define LED_STRIP_COUNT 400
#define PIN_LED_STRIP_TOP -1
#define PIN_LED_STRIP_TOP_SCK PB3
#define PIN_LED_STRIP_TOP_MISO PB4
#define LED_STRIP_TOP_COUNT 100 // the rails; JumperlOS's top strip goes on to the header and logo LEDs (145)

// True if the VIO18 rail is set to 3.3 V (what every pin above needs).
bool boardVioIs3V3( void );

// Onboard LEDs (active low handled here).
void boardLedsInit( void );
void boardLed( int pin, bool on );

#endif // BOARDPINS_H
