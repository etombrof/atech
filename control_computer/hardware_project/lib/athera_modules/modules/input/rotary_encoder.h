/**
 * @file rotary_encoder.h
 * @brief Rotary Encoder Knob Module for Athera
 *
 * 15-step rotary encoder with push-button switch and 12-LED NeoPixel
 * indicator ring. Built-in acceleration: slow turns = fine control, fast turns
 * = big jumps.
 *
 * Decoding uses the ESP32-S3 hardware pulse counter (PCNT) via ESP32Encoder.
 * Edges are counted in silicon, so the count is immune to CPU load — it never
 * drifts even when the main loop is busy pushing a display or driving other
 * LEDs, and there is no ISR to contend with other peripherals. NOTE: the
 * ESP32-S3 has only 4 PCNT units, so at most 4 knobs can track at once.
 *
 * The indicator ring shows the knob's current position as a crisp two-LED
 * "needle". It is rendered synchronously from update() on the caller's task —
 * deliberately NOT from a background task — so its NeoPixel show() is serialized
 * with any other strip in the program (e.g. a grid) and can never collide with
 * another show() on the other core (which wedges the RMT peripheral).
 *
 * Double module spanning two adjacent ports:
 * - Port 1 Line A (pin):   CLK / encoder A
 * - Port 1 Line B (pin_b): DT  / encoder B
 * - Port 2 Line A (pin_c): SW  (button, active low)
 * - Port 2 Line B (pin_d): NeoPixel ring data (12 WS2812B LEDs)
 *
 * LED chain layout (12 LEDs total, but backwards-compatible with 6-LED knobs):
 *   Chain indices 0-5 are the original 6 LEDs at even physical positions.
 *   Chain indices 6-11 are the new in-between LEDs at odd physical positions.
 *   On older 6-LED hardware, chain indices 6-11 are no-ops (data goes nowhere).
 *
 *   Around the ring (counter-clockwise from 12 o'clock):
 *     physical pos:  0   1   2   3   4   5   6   7   8   9  10  11
 *     chain idx:     0   7   1   8   2   9   3  10   4  11   5   6
 */

#ifndef ROTARY_ENCODER_MODULE_H
#define ROTARY_ENCODER_MODULE_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class ESP32Encoder;  // fwd-decl: PCNT backend, pulled in fully by the .cpp

class RotaryEncoder {
public:
    static const uint8_t RING_LEDS = 12;
    static const int32_t DETENTS_PER_REV = 15;

    // Full-quadrature raw counts per physical detent.
    static const int COUNTS_PER_DETENT = 4;
    // Hardware PCNT units available on the ESP32-S3.
    static const int PCNT_UNIT_LIMIT = 4;

    /**
     * @brief Construct rotary encoder
     * @param pinClk  GPIO for clock line / encoder A (Line A1)
     * @param pinDt   GPIO for data line / encoder B (Line B1)
     * @param pinSw   GPIO for button switch (Line A2), -1 = no button
     * @param pinRing GPIO for NeoPixel ring data (Line B2), -1 = no ring
     */
    RotaryEncoder(int pinClk, int pinDt, int pinSw = -1, int pinRing = -1);

    /** @brief Initialize encoder, button, and ring; claim a PCNT unit */
    void begin();

    /**
     * @brief Sample the encoder, poll the button, and refresh the ring.
     * MUST be called once per loop iteration.
     */
    void update();

    /** @brief Get cumulative position count (with acceleration applied) */
    int32_t getPosition();

    /** @brief Set position to a specific value */
    void setPosition(int32_t pos);

    /** @brief Reset position to zero */
    void resetPosition();

    /**
     * @brief Get direction of last rotation
     * @return 1 = clockwise, -1 = counter-clockwise, 0 = no movement
     */
    int getDirection();

    /** @brief True if knob rotated clockwise since last check */
    bool wasRotatedCW();

    /** @brief True if knob rotated counter-clockwise since last check */
    bool wasRotatedCCW();

    /** @brief Check if button is currently pressed */
    bool isPressed();

    /** @brief Check if button was just pressed (edge detection, clears after read) */
    bool wasPressed();

    /**
     * @brief Set acceleration parameters
     * @param enabled  Enable/disable acceleration (default: enabled)
     * @param maxMultiplier  Maximum step multiplier when spinning fast (default: 5)
     */
    void setAcceleration(bool enabled, int maxMultiplier = 5);

    // --- Ring indicator methods ---

    /** @brief Enable or disable the ring indicator */
    void enableRing(bool enabled);

    /** @brief Set ring indicator color (r,g,b 0-255) */
    void setRingColor(uint8_t r, uint8_t g, uint8_t b);

    /** @brief Set ring global brightness (0-255) */
    void setRingBrightness(uint8_t brightness);

    /**
     * @brief Manually override ring position (decouples from knob)
     * @param pos  Ring position as float (0.0 to 12.0, wraps). Measured around
     *             the physical ring (CCW from 12 o'clock). Pass NAN or negative
     *             to re-couple to knob position.
     */
    void setRingPosition(float pos);

private:
    int _pinClk;
    int _pinDt;
    int _pinSw;
    int _pinRing;

    // PCNT decode backend
    bool _usePcnt;                // false = no PCNT unit available (>4 knobs)
    ESP32Encoder* _enc;           // owns one PCNT unit
    int32_t _lastPolledDetent;    // last detent count consumed by update()
    static int _pcntUnitsUsed;    // running count of PCNT units handed out

    int32_t _position;            // accelerated (getPosition)
    int32_t _rawPosition;         // 1:1 with detents (drives the ring)
    int _lastDirection;
    bool _cwFlag;
    bool _ccwFlag;

    // Acceleration
    bool _accelEnabled;
    int _accelMaxMultiplier;
    unsigned long _lastStepTime;

    // Button
    bool _lastBtnState;
    bool _pressedFlag;

    // Ring
    Adafruit_NeoPixel* _ring;
    bool _ringEnabled;
    uint8_t _ringR, _ringG, _ringB;
    float _ringOverridePos;       // negative = follow knob
    int32_t _lastRingPosition;    // last position rendered (INT32_MIN = force)

    void _updateRing();
    void _renderRing(float pos);

    // Turn a signed detent delta into position + acceleration + direction flags.
    void _applyMotion(int steps);
    static int32_t _countToDetent(int64_t rawCount);
    int _calcAccelMultiplier(unsigned long elapsed);
};

#endif // ROTARY_ENCODER_MODULE_H
