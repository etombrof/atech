#!/usr/bin/env python3
"""Turn the Atech knob's serial events into a real Alt+Tab (X11, XTest).

Turning holds Alt and steps Tab / Shift+Tab; pressing the knob (or a few
seconds idle) releases Alt, which selects the highlighted window.
The middle button taps Super.
"""
import glob, sys, time
import serial
from Xlib import X, XK, display
from Xlib.ext import xtest

IDLE_COMMIT = 3.0  # seconds without a turn before Alt is released

dpy = display.Display()
ALT, SHIFT, TAB, SUPER = (dpy.keysym_to_keycode(XK.string_to_keysym(k)) for k in ('Alt_L', 'Shift_L', 'Tab', 'Super_L'))


def key(code, down):
    xtest.fake_input(dpy, X.KeyPress if down else X.KeyRelease, code)


def tab(backwards):
    if backwards:
        key(SHIFT, True)
    key(TAB, True)
    key(TAB, False)
    if backwards:
        key(SHIFT, False)
    dpy.sync()


def tap_super():
    key(SUPER, True)
    key(SUPER, False)
    dpy.sync()


def set_alt(down):
    key(ALT, down)
    dpy.sync()


def find_port():
    ports = glob.glob('/dev/serial/by-id/usb-Espressif*') + glob.glob('/dev/ttyACM*')
    return ports[0] if ports else None


def run(port):
    alt_down, last_turn = False, 0.0
    with serial.Serial(port, 115200, timeout=0.1) as ser:
        print(f'listening on {port}', flush=True)
        try:
            while True:
                line = ser.readline().decode(errors='ignore')
                if 'switch_next' in line or 'switch_prev' in line:
                    if not alt_down:
                        set_alt(True)
                        alt_down = True
                    tab(backwards='switch_prev' in line)
                    last_turn = time.monotonic()
                elif 'super_key' in line:
                    tap_super()
                elif alt_down and ('knob_press' in line or time.monotonic() - last_turn > IDLE_COMMIT):
                    set_alt(False)
                    alt_down = False
        finally:
            if alt_down:
                set_alt(False)


while True:
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    try:
        if port:
            run(port)
    except serial.SerialException as e:
        print(e, file=sys.stderr, flush=True)
    time.sleep(2)  # board unplugged or port busy: retry
