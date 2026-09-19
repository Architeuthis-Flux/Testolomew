// SPDX-License-Identifier: MIT
#ifndef UISTREAM_H
#define UISTREAM_H
// ---------------------------------------------------------------------------
// The console's port, with the screen listening in. Everything the console
// writes goes to the serial port AND into a ring of lines the log screen
// shows; everything the console reads comes from the serial port OR from
// keys the on-screen menu injects (so a menu item can run a console command,
// even one that asks for a number - the digits and Enter are queued first).
//
// This is the one class in the project that derives from something other
// than Service. A Stream is the only shape Arduino code that prints accepts,
// and JumperlOS has the same thing (Jerial, its serial fan-out; OLEDSTREAM
// mirrors the console to its OLED), so it is a deliberate exception, kept as
// small as it can be.
// ---------------------------------------------------------------------------
#include <Arduino.h>

#define UILOG_LINES 120  // lines kept
#define UILOG_WIDTH 40   // characters per line: what the LCD fits at size 1 (longer lines wrap)
#define UISTREAM_KEYS 24 // injected keys waiting to be read

class UiStream : public Stream {
  public:
    void begin( Stream* serial );

    // Print side: to the serial port and the log.
    size_t write( uint8_t c ) override;
    size_t write( const uint8_t* buffer, size_t size ) override;

    // Read side: injected keys first, then the serial port.
    int available( ) override;
    int read( ) override;
    int peek( ) override;
    void flush( ) override;

    // Queue keys for the console to read next (a menu-run command's number).
    bool inject( const char* keys );

    // The log: line n back from the newest (0 = newest). nullptr past the end.
    const char* logLine( int back ) const;
    int logCount( ) const { return count; }
    uint32_t logGeneration( ) const { return generation; } // bumps on every change
    // Off: what is printed goes to the serial port only, not into the log
    // (a screen dump or a :log listing is not something the screen should
    // show back).
    bool logToScreen = true;

  private:
    Stream* port = nullptr;
    char lines[ UILOG_LINES ][ UILOG_WIDTH + 1 ] = { };
    int head = 0;  // the line being written
    int count = 1; // lines in use, including the one being written
    int column = 0;
    uint32_t generation = 0;
    char keys[ UISTREAM_KEYS ];
    int keyHead = 0, keyCount = 0;
    void newLine( );
    void logChar( uint8_t c ); // the log side of write()
};

extern UiStream uiStream;

#endif // UISTREAM_H
