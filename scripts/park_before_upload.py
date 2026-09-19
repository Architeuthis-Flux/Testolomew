# Before `pio run -t upload`: send the console command F on the monitor port,
# so the firmware stops the sensor sampler and parks the other core (the
# V3F) in ITCM. That core has no instruction cache and fetches every
# instruction from the flash being programmed; a page program with it
# running does not complete (the Arduino core's ch32h4_park.c), which showed
# here as "Error while fastprogram: [41, 01, 01, 05]" on every other flash
# once the sampler ran on the V3F, and once as a board that needed RESET held
# through the next flash (2026-09-19). wlink resets the chip after the flash,
# which brings both cores back.
#
# If the port cannot be opened (the monitor has it, the board is not
# running the firmware), the upload goes ahead anyway.
Import("env")
import time


def park_the_other_core(source, target, env):
    port = env.GetProjectOption("monitor_port", None)
    if not port:
        print("park_before_upload: no monitor_port set, flashing as is")
        return
    try:
        import serial

        with serial.Serial(port, 115200, timeout=0.2) as s:
            s.reset_input_buffer()
            s.write(b"F")
            s.flush()
            # Wait for the firmware to say the other core is parked (up to
            # 2 s: the console runs every 5 ms, but a settings write can hold
            # the loop ~70 ms and a sensor recovery longer). One flash failed
            # with a fixed half-second wait (2026-09-19).
            seen = b""
            deadline = time.time() + 2.0
            while time.time() < deadline and b"parked" not in seen and b"did not park" not in seen:
                seen += s.read(256)
        if b"parked" in seen:
            print("park_before_upload: F sent on %s, the other core reports parked" % port)
        elif b"did not park" in seen:
            print("park_before_upload: F sent on %s but the other core did NOT park: flashing anyway, hold RESET if it fails" % port)
        else:
            print("park_before_upload: F sent on %s, no answer in 2 s (not running the firmware?): flashing anyway" % port)
    except Exception as e:
        print("park_before_upload: could not send F on %s (%s): flashing anyway; hold RESET if it fails" % (port, e))


env.AddPreAction("upload", park_the_other_core)
