# SPDX-License-Identifier: MIT
# Tile PNGs (all the same size) into one, left to right, top to bottom:
#   python3 montage.py out.png a.png b.png ... [--columns 5] [--gap 4] [--scale 1]
# `gap` pixels of dark grey lie between the tiles and around them. No labels.
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import rgb565png  # noqa: E402  (tools/rgb565png.py)

BACKGROUND = bytes((32, 32, 32))


def montage(images, columns, gap):
    """Lay (width, height, rows) images out in a grid; returns (w, h, rows)."""
    width, height = images[0][0], images[0][1]
    columns = max(1, min(columns, len(images)))
    lines = -(-len(images) // columns)  # rows of tiles
    out_w = columns * width + (columns + 1) * gap
    out_h = lines * height + (lines + 1) * gap
    blank = BACKGROUND * out_w
    rows = [blank] * out_h
    for n, (w, h, tile) in enumerate(images):
        if (w, h) != (width, height):
            raise ValueError('image %d is %dx%d, the first is %dx%d' % (n, w, h, width, height))
        x0 = gap + (n % columns) * (width + gap)
        y0 = gap + (n // columns) * (height + gap)
        for y in range(height):
            row = bytearray(rows[y0 + y])
            row[x0 * 3:(x0 + width) * 3] = tile[y]
            rows[y0 + y] = bytes(row)
    return out_w, out_h, rows


def main(argv=None):
    parser = argparse.ArgumentParser(description='tile same-sized PNGs into one PNG')
    parser.add_argument('out', help='the montage to write')
    parser.add_argument('png', nargs='+', help='the tiles, in order')
    parser.add_argument('--columns', type=int, default=5, help='tiles per row (default 5)')
    parser.add_argument('--gap', type=int, default=4, help='pixels between and around tiles (default 4)')
    parser.add_argument('--scale', type=int, default=1, help='integer upscale of the result (default 1)')
    args = parser.parse_args(argv)
    if args.gap < 0 or args.scale < 1 or args.columns < 1:
        parser.error('--gap must be >= 0, --scale and --columns >= 1')
    images = [rgb565png.read_png(p) for p in args.png]
    w, h, rows = montage(images, args.columns, args.gap)
    rgb565png.write_png(args.out, w, h, rows, scale=args.scale)
    print('%s: %dx%d, %d tiles' % (args.out, w * args.scale, h * args.scale, len(images)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
