// SPDX-License-Identifier: MIT
// The Info app: the build, the sensors, the frame rate, the menu's headroom
// and the service table, as size-1 text, refreshed once a second.
#include <Adafruit_GFX.h>

#include "Apps.h"
#include "Display.h"
#include "FastDraw.h"
#include "JumperlOS.h"
#include "MagArray.h"
#include "Menu.h"
#include "Ui.h"
#include "UiLayout.h"
#include "config.h"
#if MODULE_PROBE_LEDS
#include "ProbeLedService.h"
#endif

void infoDraw( GFXcanvas16* canvas ) {
    const int T = UI_LOG_TEXT, H = 8 * UI_LOG_TEXT;
    char line[ 48 ];
    int y = 2;
    fastText( canvas, 0, y, UI_TEXT, UI_COLOR_FRAME, "Testolomew" );
    y += UI_LINE_H + 2;
    snprintf( line, sizeof( line ), "built %s %s", __DATE__, __TIME__ );
    fastText( canvas, 0, y, T, UI_COLOR_TEXT, line );
    y += H;
    snprintf( line, sizeof( line ), "sensors %d/%d ok, %lu bus resets", magArray.sensorsOk( ), magArray.sensorCount( ), (unsigned long)magArray.busResetCount( ) );
    fastText( canvas, 0, y, T, UI_COLOR_TEXT, line );
    y += H;
    snprintf( line, sizeof( line ), "display %.0f fps, %lu yields", display.fps( ), (unsigned long)display.yields );
    fastText( canvas, 0, y, T, UI_COLOR_TEXT, line );
    y += H;
#if MODULE_PROBE_LEDS
    snprintf( line, sizeof( line ), "LEDs %s %d, chain %s, %.0f mA of %.0f", probeLeds.v5 ? "V5" : "V6", probeLeds.layout.count, probeLeds.strip ? "on" : "off", probeLeds.stripLastMa, probeLeds.stripBudgetMa( ) );
    fastText( canvas, 0, y, T, UI_COLOR_TEXT, line );
    y += H;
#endif
    snprintf( line, sizeof( line ), "menu %d/%d items, %d free", ui.shell.menu.count, MENU_MAX_ITEMS, menuFree( &ui.shell.menu ) );
    fastText( canvas, 0, y, T, UI_COLOR_TEXT, line );
    y += H;
    snprintf( line, sizeof( line ), "uptime %lu s, %lu passes", (unsigned long)( millis( ) / 1000 ), jOS.getLoopCount( ) );
    fastText( canvas, 0, y, T, UI_COLOR_TEXT, line );
    y += H + 4;
    fastText( canvas, 0, y, T, UI_COLOR_DIM, "service        runs  avg us  max us  over" );
    y += H;
    for ( uint8_t i = 0; i < jOS.getServiceCount( ) && y < LCD_HEIGHT - H; i++ ) {
        Service* s = jOS.getServiceAt( i );
        uint32_t avg = s->runs ? (uint32_t)( s->totalUs / s->runs ) : 0;
        snprintf( line, sizeof( line ), "%-11s %8lu %7lu %7lu %5lu", s->getName( ), (unsigned long)s->runs, (unsigned long)avg, (unsigned long)s->maxUs, (unsigned long)s->overruns );
        fastText( canvas, 0, y, T, s->overruns ? UI_COLOR_WARNING : UI_COLOR_TEXT, line );
        y += H;
    }
}

uint32_t infoGeneration( ) {
    return 1u + (uint32_t)( millis( ) / 1000 ); // once a second
}
