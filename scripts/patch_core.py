# Before the build: give the CH32H4 Arduino core a hook on the V3F before it
# wakes the V5F, ch32h4_v3f_before_wake(), weak, so the sketch may darken
# the LED chain first (src/probeled/LedStripDark.cpp). A WS2812 chain keeps
# its last frame through every reset of the chip, and on 2026-09-21 a lit
# cursor frame held the rail down at every wake: sixty resets in four
# seconds until a hand cut the LEDs' power. The core has no such hook, and it
# is pinned by commit (platformio.ini), so the patch is applied here, to the
# installed package, each build: idempotent, verified, and loud if the file
# no longer looks as expected. (The right home for this is upstream.)
Import("env")
import os

pkg = env.PioPlatform().get_package_dir("framework-arduinoch32h4")
path = os.path.join(pkg, "cores", "ch32h4", "main_v3f.c")
DECL = "extern void ch32h4_v3f_before_wake(void) __attribute__((weak));"
CALL = "    if (ch32h4_v3f_before_wake) {\n        ch32h4_v3f_before_wake();\n    }\n"
DECL_ANCHOR = "extern void loop1(void) __attribute__((weak));\n"
CALL_ANCHOR = '    ch32h4_console_puts("V3F: waking V5F\\n");\n'

with open(path) as f:
    src = f.read()
if DECL in src and CALL in src:
    print("patch_core: the V3F pre-wake hook is present in", path)
else:
    if DECL_ANCHOR not in src or CALL_ANCHOR not in src:
        raise SystemExit("patch_core: main_v3f.c no longer looks as expected; the V3F pre-wake hook cannot be applied - see scripts/patch_core.py")
    if DECL not in src:
        src = src.replace(DECL_ANCHOR, DECL_ANCHOR + "/* Testolomew (scripts/patch_core.py): the sketch may run something on this core\n * before the wake; it darkens the LED chain, whose lit frame otherwise holds the\n * rail down at every wake. Weak: nothing happens when the sketch has no such thing. */\n" + DECL + "\n", 1)
    if CALL not in src:
        src = src.replace(CALL_ANCHOR, CALL + CALL_ANCHOR, 1)
    with open(path, "w") as f:
        f.write(src)
    with open(path) as f:
        check = f.read()
    if DECL not in check or CALL not in check:
        raise SystemExit("patch_core: the V3F pre-wake hook did not apply - see scripts/patch_core.py")
    print("patch_core: the V3F pre-wake hook applied to", path)
