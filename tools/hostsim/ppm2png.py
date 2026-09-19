# SPDX-License-Identifier: MIT
# A binary P6 PPM (as the host simulation writes) to a PNG, 3x upscaled:
#   python3 ppm2png.py in.ppm out.png
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import rgb565png  # noqa: E402  (tools/rgb565png.py)


def main(argv):
    if len(argv) != 3:
        print('usage: python3 ppm2png.py in.ppm out.png', file=sys.stderr)
        return 2
    with open(argv[1], 'rb') as f:
        width, height, rows = rgb565png.ppm_to_rows(f.read())
    rgb565png.write_png(argv[2], width, height, rows, scale=3)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
