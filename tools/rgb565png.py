# SPDX-License-Identifier: MIT
"""RGB565 screen dumps and 8-bit RGB PNGs, standard library only.

Shared by tools/screendump.py, tools/hostsim/ppm2png.py and
tools/hostsim/montage.py. Nothing here touches the serial port.

An image is (width, height, rows): rows is a list of `height` bytes
objects, each `width * 3` bytes of RGB.
"""

import base64
import binascii
import re
import struct
import zlib

PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'

# ---- RGB565 ------------------------------------------------------------------


def rgb565_to_rgb(v):
    """One RGB565 value (rrrrrggggggbbbbb) to an 8-bit (r, g, b) tuple."""
    return (((v >> 11) & 31) * 255 // 31,
            ((v >> 5) & 63) * 255 // 63,
            (v & 31) * 255 // 31)


_rgb565_table = None


def _rgb565_bytes(v):
    """Three RGB bytes for an RGB565 value, from a table built on first use."""
    global _rgb565_table
    if _rgb565_table is None:
        _rgb565_table = [bytes(rgb565_to_rgb(i)) for i in range(65536)]
    return _rgb565_table[v]


# ---- PNG writing -------------------------------------------------------------


def _png_chunk(kind, body):
    return (struct.pack('>I', len(body)) + kind + body
            + struct.pack('>I', zlib.crc32(kind + body) & 0xffffffff))


def write_png(path, width, height, rows, scale=1):
    """Write an 8-bit RGB PNG: filter 0 on every scanline, one IDAT.

    `scale` is an integer nearest-neighbour upscale (3 reproduces the old
    ppm2png.py output).
    """
    scale = int(scale)
    if scale < 1:
        raise ValueError('scale must be at least 1')
    if len(rows) != height:
        raise ValueError('expected %d rows, got %d' % (height, len(rows)))
    raw = bytearray()
    for row in rows:
        if len(row) != width * 3:
            raise ValueError('row has %d bytes, expected %d' % (len(row), width * 3))
        if scale == 1:
            line = b'\x00' + bytes(row)
        else:
            line = bytearray(b'\x00')
            for x in range(width):
                line += row[x * 3:x * 3 + 3] * scale
            line = bytes(line)
        raw += line * scale
    ihdr = struct.pack('>IIBBBBB', width * scale, height * scale, 8, 2, 0, 0, 0)
    with open(path, 'wb') as f:
        f.write(PNG_SIGNATURE
                + _png_chunk(b'IHDR', ihdr)
                + _png_chunk(b'IDAT', zlib.compress(bytes(raw), 9))
                + _png_chunk(b'IEND', b''))


# ---- PNG reading -------------------------------------------------------------


def _paeth(a, b, c):
    p = a + b - c
    pa = abs(p - a)
    pb = abs(p - b)
    pc = abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def _unfilter(kind, cur, prev, bpp):
    """Reconstruct one scanline (filter byte already removed) in place."""
    n = len(cur)
    if kind == 0:
        return
    if kind == 1:  # sub
        for i in range(bpp, n):
            cur[i] = (cur[i] + cur[i - bpp]) & 255
    elif kind == 2:  # up
        for i in range(n):
            cur[i] = (cur[i] + prev[i]) & 255
    elif kind == 3:  # average
        for i in range(n):
            a = cur[i - bpp] if i >= bpp else 0
            cur[i] = (cur[i] + ((a + prev[i]) >> 1)) & 255
    elif kind == 4:  # paeth
        for i in range(n):
            if i >= bpp:
                a = cur[i - bpp]
                c = prev[i - bpp]
            else:
                a = c = 0
            cur[i] = (cur[i] + _paeth(a, prev[i], c)) & 255
    else:
        raise ValueError('unknown PNG filter type %d' % kind)


def read_png(path):
    """Read an 8-bit RGB (colour type 2), non-interlaced PNG.

    Any of the five scanline filters and several IDAT chunks are fine;
    palette, alpha, greyscale, 16-bit and interlaced files raise ValueError.
    """
    with open(path, 'rb') as f:
        data = f.read()
    if data[:8] != PNG_SIGNATURE:
        raise ValueError('%s: not a PNG' % path)
    pos = 8
    width = height = None
    idat = bytearray()
    while pos + 8 <= len(data):
        length, kind = struct.unpack('>I4s', data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + length]
        pos += 12 + length
        if kind == b'IHDR':
            width, height, depth, colour, comp, filt, interlace = struct.unpack('>IIBBBBB', body)
            if depth != 8 or colour != 2:
                raise ValueError('%s: only 8-bit RGB PNGs are handled (depth %d, colour type %d)'
                                 % (path, depth, colour))
            if interlace != 0:
                raise ValueError('%s: interlaced PNGs are not handled' % path)
            if comp != 0 or filt != 0:
                raise ValueError('%s: unknown compression/filter method' % path)
        elif kind == b'IDAT':
            idat += body
        elif kind == b'IEND':
            break
        # every other chunk (pHYs, sRGB, tEXt, ...) is skipped
    if width is None:
        raise ValueError('%s: no IHDR' % path)
    raw = zlib.decompress(bytes(idat))
    stride = width * 3
    if len(raw) < height * (stride + 1):
        raise ValueError('%s: image data is short' % path)
    rows = []
    prev = bytearray(stride)
    for y in range(height):
        start = y * (stride + 1)
        kind = raw[start]
        cur = bytearray(raw[start + 1:start + 1 + stride])
        _unfilter(kind, cur, prev, 3)
        rows.append(bytes(cur))
        prev = cur
    return width, height, rows


# ---- console frames ----------------------------------------------------------


def _find_frame(text, name):
    """The lines of the last complete `name{` ... `}` frame in text, or of
    the last unfinished one; None when there is no opening line."""
    lines = text.splitlines()
    opening = name + '{'
    best = None      # lines of the last complete frame
    unfinished = None
    i = 0
    while i < len(lines):
        if not lines[i].strip().endswith(opening):
            i += 1
            continue
        j = i + 1
        while j < len(lines) and lines[j].strip() != '}':
            j += 1
        if j < len(lines):
            best = lines[i + 1:j]
            unfinished = None
            i = j + 1
        else:
            unfinished = lines[i + 1:]
            i = len(lines)
    if best is not None:
        return best
    return unfinished


_HEADER_RE = re.compile(r'^\s*w\s+(\d+)\s+h\s+(\d+)\s+step\s+(\d+)\s+(rle|b64)\s*$')
_ROW_RE = re.compile(r'^\s*(\d+):(.*)$')
_RUN_RE = re.compile(r'^(\d+)x([0-9a-fA-F]{4})$')


def _decode_rle_row(payload, width):
    """The RGB bytes of one run-length row, cut at width and padded black."""
    out = bytearray()
    for token in payload.split(','):
        m = _RUN_RE.match(token.strip())
        if not m:
            continue  # a run cut short by the line buffer, or an empty tail
        count = int(m.group(1))
        pixel = _rgb565_bytes(int(m.group(2), 16))
        room = width - len(out) // 3
        if room <= 0:
            break
        out += pixel * min(count, room)
    out += b'\x00' * (width * 3 - len(out))
    return bytes(out)


def _decode_b64_row(payload, width):
    """The RGB bytes of one base64 row (little-endian RGB565), padded black."""
    payload = ''.join(payload.split())
    payload = payload[:len(payload) // 4 * 4]  # a truncated tail is dropped
    try:
        raw = base64.b64decode(payload)
    except (ValueError, binascii.Error):
        raw = b''
    out = bytearray()
    for i in range(0, len(raw) - 1, 2):
        if len(out) >= width * 3:
            break
        out += _rgb565_bytes(raw[i] | (raw[i + 1] << 8))
    out += b'\x00' * (width * 3 - len(out))
    return bytes(out)


def decode_dump(text):
    """Decode the last `screen:dump{` frame in text.

    Returns (width, height, rows, header): width is the samples per row
    (ceil(w / step)), height the rows expected (floor(h / step)), rows the
    RGB bytes of each, black where the dump has none; header is a dict
    with w, h, step and kind ('rle' or 'b64').
    """
    body = _find_frame(text, 'screen:dump')
    if body is None:
        raise ValueError('no screen:dump{ frame in the text')
    header = None
    first = 0
    for k, line in enumerate(body):
        m = _HEADER_RE.match(line)
        if m:
            header = {'w': int(m.group(1)), 'h': int(m.group(2)),
                      'step': int(m.group(3)), 'kind': m.group(4)}
            first = k + 1
            break
    if header is None:
        raise ValueError('screen:dump{ frame has no header line')
    step = max(1, header['step'])
    width = -(-header['w'] // step)
    height = header['h'] // step
    black = b'\x00' * (width * 3)
    rows = [black] * height
    decode_row = _decode_rle_row if header['kind'] == 'rle' else _decode_b64_row
    for line in body[first:]:
        m = _ROW_RE.match(line)
        if not m:
            continue  # something else the console printed between rows
        y = int(m.group(1))
        if y % step != 0 or y // step >= height:
            continue
        rows[y // step] = decode_row(m.group(2).rstrip('\r\n'), width)
    return width, height, rows, header


def find_ascii(text):
    """The body of the last `screen:ascii{` frame, header line included,
    joined with newlines; None when there is none."""
    body = _find_frame(text, 'screen:ascii')
    if body is None:
        return None
    return '\n'.join(line.rstrip('\r\n') for line in body)


# ---- PPM ---------------------------------------------------------------------


def ppm_to_rows(data):
    """A binary P6 PPM (maxval 255) to (width, height, rows)."""
    if data[:2] != b'P6':
        raise ValueError('not a P6 PPM')
    # Header tokens: P6, width, height, maxval, separated by whitespace and
    # possibly # comments; one whitespace byte after maxval, then the pixels.
    pos = 2
    fields = []
    while len(fields) < 3:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] not in (b'\n', b'\r'):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        if start == pos:
            raise ValueError('short PPM header')
        fields.append(int(data[start:pos]))
    pos += 1  # the single whitespace byte after maxval
    width, height, maxval = fields
    if maxval != 255:
        raise ValueError('PPM maxval %d is not 255' % maxval)
    pixels = data[pos:pos + width * height * 3]
    if len(pixels) < width * height * 3:
        raise ValueError('PPM pixel data is short')
    rows = [bytes(pixels[y * width * 3:(y + 1) * width * 3]) for y in range(height)]
    return width, height, rows
