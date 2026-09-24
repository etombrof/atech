# control_computer

A physical Alt+Tab knob and Super button for Linux, built on an Atech ESP32-S3 board.

- **Turn the knob**: step through open windows, like holding Alt and pressing Tab.
- **Press the knob**: switch to the highlighted window. It also switches by itself 3 seconds after your last turn.
- **Press the middle button**: tap the Super key, which opens the GNOME overview.

## Files

- `firmware.cpp`: firmware for the board (rotary knob on ports 1+2, button, ST7735 display). It sends
  `switch_next`, `switch_prev`, `knob_press` and `super_key` events as JSON lines over USB serial.
- `run.py`: host script. It reads those events and turns them into real key presses.

## Run

Needs Linux on X11 (not Wayland).

```sh
pip install pyserial python-xlib
python3 run.py            # finds the board automatically (/dev/ttyACM*)
python3 run.py /dev/ttyACM0
```

Your user must be in the `dialout` group to open the serial port. Close any other program that has the
port open first, such as the Atech dashboard's serial connection.

## Why it feels like real Alt+Tab

1. **Alt is held down.** Sending a full `alt+Tab` on every step presses and releases Alt each time, which just
   flips between two windows. The script holds Alt while you turn and sends only Tab or Shift+Tab.
2. **One event per detent.** The firmware compares the knob's position on every loop and sends one event for
   each step it moved. Fast turns no longer get merged into a single event.
3. **Acceleration off.** With acceleration on, a fast turn jumped an unpredictable number of windows. Now one
   click moves one window.
4. **No process per key.** Keys go straight to X through XTest (python-xlib) instead of starting an `xdotool`
   process each time, which cost about 10–30 ms per step.
