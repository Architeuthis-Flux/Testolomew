// The core's EEPROM library on the host: a RAM mirror that "commits" to nowhere.
#pragma once
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
class EEPROMClass {
  public:
    bool begin( size_t size = 4096 ) { _size = size; _mirror = (uint8_t*)calloc( size, 1 ); return _mirror != nullptr; }
    uint8_t read( int a ) { return a >= 0 && a < (int)_size ? _mirror[ a ] : 0; }
    void write( int a, uint8_t v ) { if ( a >= 0 && a < (int)_size ) _mirror[ a ] = v; }
    bool commit( ) { commits++; return true; }
    template <typename T> T& get( int a, T& t ) { if ( a >= 0 && a + (int)sizeof( T ) <= (int)_size ) memcpy( &t, _mirror + a, sizeof( T ) ); return t; }
    template <typename T> const T& put( int a, const T& t ) { if ( a >= 0 && a + (int)sizeof( T ) <= (int)_size ) memcpy( _mirror + a, &t, sizeof( T ) ); return t; }
    int commits = 0;
  private:
    size_t _size = 0; uint8_t* _mirror = nullptr;
};
extern EEPROMClass EEPROM;
