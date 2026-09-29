// SPDX-License-Identifier: MIT
#ifndef SETTINGS_H
#define SETTINGS_H
// ---------------------------------------------------------------------------
// Settings that survive a reboot.
//
// Everything the on-screen menu can edit - every NUMBER, TOGGLE and CHOICE
// item, keyed "<page>/<label>" - and the row counter's anchors are written
// as text ("smoothing/view Hz=1.5") into the flash tail through the core's
// EEPROM library (two 8 KB pages used in turn, so a reset mid-write loses
// nothing), and put back at boot before the modules run. Any item added to
// the menu is saved from then on with no other change; a renamed item
// starts from its default (its old record is dropped at the next save).
//
// Saving is by watching, not by hooks: the values are serialised (a few
// records every SETTINGS_CHECK_MS, a full text about once a second, so no
// one run holds the loop) and compared with what was saved; a change that
// has stood UNCHANGED for SETTINGS_SETTLE_MS is committed (one that keeps
// changing waits). So a console command that changes a
// setting (S17, t12, u, R30...) is saved the same as a menu edit, and a
// lever being nudged twenty times is written once. A commit erases and
// programs a flash page, which holds the loop for a few tens of ms; that is
// why it waits for the values to settle. The erase is also a burst of
// current on the 3.3 V rail, and with the LED chain lit around a probe on
// the board it browned the board out (2026-09-21: the menu used with the
// probe near); so `aroundWrite` lets main.cpp put the board in the dark for
// it (the chain cleared and its last frame waited out), and back after.
//
// Not saved: row mode, the V5 console stream, the chain on/off and the play
// mode. Those are modes, and the board boots into row mode (RowCounter.h),
// quiet, with the chain on; a reset leaves them alone.
//
// Console: s = what is saved (and whether it is what is running now),
// Z = reset: every setting back to its compile-time default and the row
// anchors forgotten, now, and that saved (the menu's "reset settings").
// ---------------------------------------------------------------------------
#include <Arduino.h>

#include "JumperlOS.h"
#include "Menu.h"

#define SETTINGS_SIZE 4096 // of the EEPROM mirror (the core allows up to 8 KB - 16)
#define SETTINGS_TEXT_MAX ( SETTINGS_SIZE - 8 )
// The serialisation is ~6 ms of float formatting in all, which held the loop
// (and made the LED chain late) every check: it is done in steps, so many
// records a run, at this period - a full pass about every second.
#define SETTINGS_CHECK_MS 100
#define SETTINGS_STEPS_PER_RUN 16
#define SETTINGS_SETTLE_MS 2000
#define SETTINGS_SETTLE_OPEN_MS 30000 // ...but while the menu or a tweak is open a change waits this long instead: the write holds the loop ~70 ms, a hitch on the LEDs that read as "the setting applying later" while a lever was being tuned (2026-09-28); the panes closing writes it after the plain settle
#define SETTINGS_MAGIC 0x54534554u // "TSET"
// Bumped when the compile-time defaults of the tuning pages (tracker,
// smoothing, and play/touch) change for the better: saved values of those from an
// older tuning are then left out at boot, so the new defaults take, and the
// next write carries the new version. The other pages are always kept.
#define SETTINGS_TUNING_VERSION 6 // 6 (2026-09-28 evening): the bench's tracking levers again, the shaft's back at 3.6 / 1 (0.1 / 0 on the board was the far dance's workaround, and the shaft filter is keyed on the fit's axis bar now). 3 (2026-09-19): play/touch 1 -> 2 mm 5 (2026-09-27): every tracking lever is the bench's setting now

class SettingsService : public Service {
  public:
    static SettingsService& getInstance( );

    SettingsService( const SettingsService& ) = delete;
    SettingsService& operator=( const SettingsService& ) = delete;

    // Read the saved settings into the menu's values (through the items'
    // setters, so a module's value follows) and the anchors into the row
    // counter. Call after the menu is built and every module has begun:
    // the defaults are what the items read then. Returns how many records
    // were applied, -1 if nothing valid was saved.
    int begin( Menu* menu );

    ServiceStatus service( ) override;
    const char* getName( ) const override { return "Settings"; }
    ServicePriority getPriority( ) const override { return ServicePriority::LOW; }
    uint32_t periodUs( ) const override { return SETTINGS_CHECK_MS * 1000; }

    // Write now, whatever the settle timer says. true if a page was written.
    bool saveNow( );
    // Everything back to its default (the values as they were before the
    // saved ones went in at boot), the row anchors forgotten, and that
    // saved. The modes the modules boot with (row mode, the chain) are not
    // touched: they are not in the text.
    void reset( Stream* out );
    // What is saved, line by line, and whether it matches what is running.
    void print( Stream* out );

    // Called just before a page is erased (true) and right after it is
    // programmed (false): main.cpp clears the LED chain for the erase and
    // brings it back (the header comment). Nothing is called if unset.
    void ( *aroundWrite )( bool starting ) = nullptr;

    uint32_t saves = 0; // pages written since boot
    uint32_t writeFailures = 0;
    int loaded = -1; // records applied at begin()

  private:
    SettingsService( ) = default;

    Menu* menu = nullptr;
    char saved[ SETTINGS_TEXT_MAX ];    // the text as last written (or read)
    char current[ SETTINGS_TEXT_MAX ];  // the text as it would be written now
    char defaults[ SETTINGS_TEXT_MAX ]; // the text as it was before anything saved went in: the compile-time values
    char pending[ SETTINGS_TEXT_MAX ];  // the changed text whose settle time is running
    bool valid = false;                 // `saved` came from the flash and checked out
    uint32_t changedSinceMs = 0;        // 0 = current == saved
    bool wasGone = true;                // the probe was absent at the last check

    // A serialisation in progress: which record is next (0 = the header, then
    // the menu items, the zeros, the magnet, the anchors), and the text so far.
    int walkStep = 0;
    int walkLength = 0;
    bool walking = false;

    int record( int step, char* out, int size );
    int serialise( char* out, int size );                 // all at once (saves, the report)
    bool serialiseSome( char* out, int size, int steps ); // a few records more; true when the text is complete
    int apply( const char* text );
    bool readFlash( );
    bool writeFlash( const char* text );
};

extern SettingsService& settings;

#endif // SETTINGS_H
