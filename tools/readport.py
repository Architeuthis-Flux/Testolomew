# capture the port, one line per row with the host time (seconds) in front, written as it comes
import sys, time, os, termios, select
port, secs, outfile = sys.argv[1], float(sys.argv[2]), sys.argv[3]
send = sys.argv[4].encode() if len(sys.argv) > 4 else b''
fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
a[4] = a[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, a)
if send:
    time.sleep(0.2); os.write(fd, send)
t0 = time.time(); end = t0 + secs; buf = b''
with open(outfile, 'a', buffering=1) as f:
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try: buf += os.read(fd, 4096)
            except BlockingIOError: pass
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                f.write('%7.1f %s\n' % (time.time() - t0, line.decode('utf-8', 'replace').rstrip('\r')))
os.close(fd)
