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
#define PIN_MAG_VCC_CANDIDATES { PC12, PA13, PD1, PD7, PC8, PC7, PC6 }
#define PIN_MAG_VCC_CANDIDATE_NAMES { "PC12", "PA13", "PD1", "PD7", "PC8", "PC7", "PC6" }

// Each set of four has its ground on a GPIO too; they are driven low and left.
#define PIN_MAG_GND_A PA15
#define PIN_MAG_GND_B PD5

// True if the VIO18 rail is set to 3.3 V (what every pin above needs).
bool boardVioIs3V3( void );

// Onboard LEDs (active low handled here).
void boardLedsInit( void );
void boardLed( int pin, bool on );

#endif // BOARDPINS_H
