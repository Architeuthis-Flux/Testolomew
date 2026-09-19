import sys, zlib, struct
def conv(src, dst, k=3):
    d = open(src,'rb').read()
    parts = d.split(b'\n', 3)
    w, h = map(int, parts[1].split()); px = parts[3]
    rows = []
    for y in range(h):
        row = bytearray()
        for x in range(w):
            row += px[(y*w+x)*3:(y*w+x)*3+3] * k
        rows += [bytes([0]) + bytes(row)] * k
    raw = b''.join(rows)
    def chunk(t, c): return struct.pack('>I', len(c)) + t + c + struct.pack('>I', zlib.crc32(t + c) & 0xffffffff)
    open(dst,'wb').write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w*k, h*k, 8, 2, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))
conv(sys.argv[1], sys.argv[2])
