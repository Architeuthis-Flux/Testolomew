#pragma once
class Print {
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t* b, size_t n) { size_t k = 0; while (n--) k += write(*b++); return k; }
    size_t print(const char* s) { size_t n = 0; while (*s) n += write((uint8_t)*s++); return n; }
    size_t print(char c) { return write((uint8_t)c); }
    size_t print(int v) { char b[16]; snprintf(b, sizeof b, "%d", v); return print(b); }
    size_t print(unsigned long v) { char b[24]; snprintf(b, sizeof b, "%lu", v); return print(b); }
    size_t print(uint32_t v) { return print((unsigned long)v); }
    size_t print(double v, int d = 2) { char b[32]; snprintf(b, sizeof b, "%.*f", d, v); return print(b); }
    size_t println() { return print("\n"); }
    size_t println(const char* s) { return print(s) + println(); }
    size_t println(int v) { return print(v) + println(); }
    size_t println(unsigned long v) { return print(v) + println(); }
    size_t println(long v) { return print((int)v) + println(); }
};
