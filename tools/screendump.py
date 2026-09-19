# SPDX-License-Identifier: MIT
"""Grab the board's screen over the console into a PNG or a text file.

  python3 tools/screendump.py <port> out.png [--b64] [--step N] [--timeout S] [--baud 115200]
  python3 tools/screendump.py <port> out.txt --ascii [--colour]
  python3 tools/screendump.py --decode captured.txt out.png
  python3 tools/screendump.py --decode captured.txt out.txt --ascii

Sends `:screen:dump rle|b64 [step]` (or `:screen:ascii [colour]`), reads
the `screen:dump{` ... `}` frame that answers, and decodes it with
tools/rgb565png.py. The PNG is (samples per row) x (rows) pixels: 240x240
at step 1, 120x120 at step 2. `--decode` reads a captured text file
instead of the port (the raw form as the board sends it: what --keep
saves, not tools/readport.py's timestamped lines).
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rgb565png  # noqa: E402


def capture(port, baud, command, opening, timeout):
    """Send one verb line and read until its frame closes or timeout runs out.

    Returns (text, complete): everything received, decoded as utf-8, and
    whether the closing `}` line was seen. Raises RuntimeError on an err{}
    answer that arrives before the frame opens.
    """
    import serial  # pyserial, only needed on the port

    ser = serial.Serial(port, baud, timeout=0.2)
    try:
        ser.reset_input_buffer()
        ser.write(command.encode('ascii'))
        ser.flush()
        buf = bytearray()
        checked = 0  # every line before this offset has been looked at
        opened = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf += chunk
            end = buf.rfind(b'\n')
            if end < checked:
                continue  # no newly completed line yet
            for line in buf[checked:end + 1].splitlines():
                line = line.strip()
                if not opened:
                    if line.endswith(opening):
                        opened = True
                    elif line.startswith(b'err{') and line.endswith(b'}'):
                        raise RuntimeError('the board answered %s' % line.decode('utf-8', 'replace'))
                elif line == b'}':
                    return buf.decode('utf-8', 'replace'), True
            checked = end + 1
        return buf.decode('utf-8', 'replace'), False
    finally:
        ser.close()


def keep_path(out):
    stem = os.path.splitext(out)[0]
    path = stem + '.txt'
    return path if os.path.abspath(path) != os.path.abspath(out) else stem + '.raw.txt'


def main(argv=None):
    parser = argparse.ArgumentParser(description='dump the screen of the board over its console',
                                     usage='%(prog)s <port> out.png|out.txt [options]\n'
                                           '       %(prog)s --decode captured.txt out.png|out.txt [--ascii]')
    parser.add_argument('port', nargs='?', help='serial port (or the output file with --decode)')
    parser.add_argument('out', nargs='?', help='out.png, or out.txt with --ascii')
    parser.add_argument('--decode', metavar='FILE', help='decode a captured text file instead of a port')
    parser.add_argument('--ascii', action='store_true', help=':screen:ascii, saved as text')
    parser.add_argument('--colour', action='store_true', help='hue letters in the ascii dump')
    parser.add_argument('--b64', action='store_true', help='base64 rows instead of run-length')
    parser.add_argument('--step', type=int, help='sample every Nth row and pixel (1-8)')
    parser.add_argument('--timeout', type=float, default=30.0, help='seconds to wait for the frame (default 30)')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--keep', action='store_true', help='also save the raw text next to the output')
    args = parser.parse_args(argv)

    if args.decode:
        if args.out is not None or args.port is None:
            parser.error('--decode takes one file to write: --decode captured.txt out.png')
        out = args.port
    else:
        if args.port is None or args.out is None:
            parser.error('a serial port and an output file are needed')
        out = args.out
    if args.step is not None and not 1 <= args.step <= 8:
        parser.error('--step must be 1 to 8')
    if args.colour and not args.ascii:
        parser.error('--colour goes with --ascii')
    if (args.b64 or args.step) and args.ascii:
        parser.error('--b64 and --step go with a pixel dump, not --ascii')

    if args.ascii:
        command = ':screen:ascii' + (' colour' if args.colour else '') + '\r'
        opening = b'screen:ascii{'
    else:
        command = ':screen:dump ' + ('b64' if args.b64 else 'rle')
        if args.step:
            command += ' %d' % args.step
        command += '\r'
        opening = b'screen:dump{'

    started = time.monotonic()
    if args.decode:
        with open(args.decode, 'r', encoding='utf-8', errors='replace') as f:
            text = f.read()
        complete = True
    else:
        try:
            text, complete = capture(args.port, args.baud, command, opening, args.timeout)
        except ImportError:
            print('pyserial is needed to talk to a port: pip install pyserial', file=sys.stderr)
            return 1
        except (RuntimeError, OSError) as e:  # an err{} answer, or the port would not open
            print(str(e), file=sys.stderr)
            return 1
        if args.keep:
            with open(keep_path(out), 'w', encoding='utf-8') as f:
                f.write(text)
        if opening.decode('ascii') not in text:
            print('no %s frame arrived from %s in %.0f s (%d bytes received)'
                  % (opening.decode('ascii'), args.port, args.timeout, len(text)), file=sys.stderr)
            return 1
    seconds = time.monotonic() - started

    if args.ascii:
        body = rgb565png.find_ascii(text)
        if body is None:
            print('no screen:ascii{ frame in the text', file=sys.stderr)
            return 1
        with open(out, 'w', encoding='utf-8') as f:
            f.write(body + '\n')
        note = '' if complete else ' (frame cut short)'
        print('%s: %d lines in %.1f s%s' % (out, body.count('\n') + 1, seconds, note))
        return 0

    try:
        width, height, rows, header = rgb565png.decode_dump(text)
    except ValueError as e:
        print(str(e), file=sys.stderr)
        return 1
    rgb565png.write_png(out, width, height, rows)
    note = '' if complete else ' (frame cut short)'
    print('%s: %dx%d pixels (%s, step %d) in %.1f s%s'
          % (out, width, height, header['kind'], header['step'], seconds, note))
    return 0


if __name__ == '__main__':
    sys.exit(main())
