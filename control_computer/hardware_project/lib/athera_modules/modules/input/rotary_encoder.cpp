/**
 * @file rotary_encoder.cpp
 * @brief Rotary Encoder Knob Module Implementation
 *
 * Hardware (PCNT) full-quadrature decoding with acceleration.
 *   - The ESP32-S3 pulse counter counts every A/B edge in silicon, so no step
 *     is ever missed regardless of loop timing or CPU load (display pushes,
 *     other LED strips, wifi, etc.). update() samples the counter each loop.
 *   - The ring is rendered synchronously in update() — no background task — so
 *     its NeoPixel show() is serialized with the rest of the program and cannot
 *     collide with another strip's show() on the other core.
 *
 * Acceleration: time between detents determines a multiplier.
 * Slow turn  = 1x (fine control)
 * Fast spin  = up to 5x (quick traversal)
 *
 * Ring indicator: 12 NeoPixels. A crisp two-LED "needle" tracks the knob angle.
 * 15 detents per revolution.
 */

#include "rotary_encoder.h"
#include <ESP32Encoder.h>
#include <math.h>

// Acceleration thresholds (microseconds between detents)
static const unsigned long ACCEL_SLOW_US  = 120000;  // ~8 steps/sec — no acceleration
static const unsigned long ACCEL_FAST_US  =  25000;  // ~40 steps/sec — max acceleration
static const unsigned long ACCEL_RANGE_US = ACCEL_SLOW_US - ACCEL_FAST_US;

// Map physical position around the ring (0=12 o'clock, going CCW) to chain index.
//   physical pos: 0  1  2  3  4  5  6  7  8  9 10 11
//   chain idx:    0  7  1  8  2  9  3 10  4 11  5  6
// Chain indices 0-5 are the original 6 LEDs at even physical positions; 6-11 are
// the in-between LEDs at odd positions. On 6-LED hardware, 6-11 go nowhere.
static inline uint8_t physicalToChain(uint8_t physical) {
    if ((physical & 1) == 0) {
        return physical >> 1;  // even -> original LEDs 0-5
    }
    return 6 + (((physical + 1) >> 1) % 6);  // odd -> in-between LEDs 6-11
}

int RotaryEncoder::_pcntUnitsUsed = 0;

RotaryEncoder::RotaryEncoder(int pinClk, int pinDt, int pinSw, int pinRing)
    : _pinClk(pinClk)
    , _pinDt(pinDt)
    , _pinSw(pinSw)
    , _pinRing(pinRing)
    , _usePcnt(false)
    , _enc(nullptr)
    , _lastPolledDetent(0)
    , _position(0)
    , _rawPosition(0)
    , _lastDirection(0)
    , _cwFlag(false)
    , _ccwFlag(false)
    , _accelEnabled(true)
    , _accelMaxMultiplier(5)
    , _lastStepTime(0)
    , _lastBtnState(false)
    , _pressedFlag(false)
    , _ring(nullptr)
    , _ringEnabled(true)
    , _ringR(255)
    , _ringG(255)
    , _ringB(255)
    , _ringOverridePos(-1.0f)
    , _lastRingPosition(INT32_MIN)
{
}

void RotaryEncoder::begin() {
    if (_pinSw >= 0) {
        pinMode(_pinSw, INPUT_PULLUP);
    }

    // Initialize ring if pin provided
    if (_pinRing >= 0) {
        _ring = new Adafruit_NeoPixel(RING_LEDS, _pinRing, NEO_GRB + NEO_KHZ800);
        _ring->begin();
        _ring->setBrightness(50);  // ~20% default brightness
        _ring->clear();
        _ring->show();
        Serial.print("[RotaryEncoder] Ring initialized on pin ");
        Serial.println(_pinRing);
    }

    _lastStepTime = micros();

    // Claim a hardware PCNT unit for full-quadrature decoding.
    if (_pcntUnitsUsed < PCNT_UNIT_LIMIT) {
        _usePcnt = true;
        _pcntUnitsUsed++;

        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        _enc = new ESP32Encoder();
        _enc->attachFullQuad(_pinClk, _pinDt);  // count all 4 edges per cycle
        _enc->setFilter(1023);                  // hardware glitch filter (~12.8us)
        _enc->clearCount();
        _lastPolledDetent = 0;

        Serial.print("[RotaryEncoder] PCNT on pins ");
        Serial.print(_pinClk); Serial.print("/"); Serial.println(_pinDt);
    } else {
        // ESP32-S3 has only 4 PCNT units. A 5th+ knob can't be counted.
        Serial.println("[RotaryEncoder] WARNING: no free PCNT unit (>4 knobs) — this knob will not track");
    }
}

// Symmetric integer floor-division so the detent count is symmetric around zero
// (-1 detent at raw count -COUNTS_PER_DETENT, not -1).
int32_t RotaryEncoder::_countToDetent(int64_t raw) {
    if (raw >= 0) return (int32_t)(raw / COUNTS_PER_DETENT);
    return (int32_t)(-(((-raw) + COUNTS_PER_DETENT - 1) / COUNTS_PER_DETENT));
}

int RotaryEncoder::_calcAccelMultiplier(unsigned long elapsed) {
    if (elapsed >= ACCEL_SLOW_US) return 1;
    if (elapsed <= ACCEL_FAST_US) return _accelMaxMultiplier;

    // Linear interpolation: map elapsed [FAST..SLOW] to multiplier [max..1]
    unsigned long fromFast = elapsed - ACCEL_FAST_US;
    int range = _accelMaxMultiplier - 1;
    return _accelMaxMultiplier - (int)(fromFast * range / ACCEL_RANGE_US);
}

// A signed detent delta (from the polled counter) updates the position counters,
// acceleration, and direction flags.
void RotaryEncoder::_applyMotion(int steps) {
    if (steps == 0) return;

    unsigned long now = micros();
    unsigned long elapsed = now - _lastStepTime;
    _lastStepTime = now;

    int dir = (steps > 0) ? 1 : -1;
    int mag = (steps > 0) ? steps : -steps;

    // Several detents can arrive in one poll, so use the average per-detent time
    // for the acceleration multiplier (not the whole loop interval).
    unsigned long perStep = (mag > 1) ? (elapsed / (unsigned long)mag) : elapsed;
    int multiplier = _accelEnabled ? _calcAccelMultiplier(perStep) : 1;

    _position += steps * multiplier;  // accelerated (getPosition)
    _rawPosition += steps;            // 1:1 with physical detents (ring)
    _lastDirection = dir;

    if (dir > 0) _cwFlag = true;
    else _ccwFlag = true;
}

void RotaryEncoder::update() {
    // Sample the hardware counter and feed any detent delta into the position.
    if (_usePcnt && _enc) {
        int32_t detent = _countToDetent(_enc->getCount());
        int32_t delta = detent - _lastPolledDetent;
        if (delta != 0) {
            _lastPolledDetent = detent;
            _applyMotion((int)delta);
        }
    }

    // Button edge detection
    if (_pinSw >= 0) {
        bool currentBtn = (digitalRead(_pinSw) == LOW);  // Active low
        if (currentBtn && !_lastBtnState) {
            _pressedFlag = true;
        }
        _lastBtnState = currentBtn;
    }

    // Render the ring synchronously on this task — serialized with any other
    // NeoPixel strip in the program, so the two show() calls never collide.
    _updateRing();
}

void RotaryEncoder::_updateRing() {
    if (!_ring || !_ringEnabled) return;

    int32_t currentPos = _rawPosition;  // raw (non-accelerated) position

    // Skip if nothing changed (and not forced / no override). show() is
    // comparatively expensive, so don't repaint an identical frame.
    if (_ringOverridePos < 0 && currentPos == _lastRingPosition) return;
    _lastRingPosition = currentPos;

    float ringPos;
    if (_ringOverridePos >= 0) {
        ringPos = _ringOverridePos;
    } else {
        // DETENTS_PER_REV detents == one full lap. Increasing knob position ->
        // increasing physical position (matches the calibrated standalone; do
        // NOT reverse — that swaps CW/CCW).
        int32_t wrapped = currentPos % DETENTS_PER_REV;
        if (wrapped < 0) wrapped += DETENTS_PER_REV;
        ringPos = (float)wrapped * (float)RING_LEDS / (float)DETENTS_PER_REV;
    }

    _renderRing(ringPos);
}

void RotaryEncoder::_renderRing(float pos) {
    // Wrap pos into [0, RING_LEDS)
    pos = fmodf(pos, (float)RING_LEDS);
    if (pos < 0) pos += (float)RING_LEDS;

    // Crisp two-LED "needle": the pointer sits between the two nearest physical
    // positions, brightness split by how far between them it is, so the dot
    // follows the knob's true angle even though there are more detents than LEDs.
    // Writes go to the INTERLEAVED chain index for each physical position.
    // (Matches the calibrated standalone knob render.)
    uint32_t newColors[RING_LEDS];
    for (int i = 0; i < RING_LEDS; i++) newColors[i] = 0;

    int base = (int)floorf(pos);
    float frac = pos - (float)base;          // 0.0 at base LED, ->1.0 toward next
    int next = (base + 1) % RING_LEDS;

    newColors[physicalToChain(base)] = _ring->Color(
        (uint8_t)(_ringR * (1.0f - frac)),
        (uint8_t)(_ringG * (1.0f - frac)),
        (uint8_t)(_ringB * (1.0f - frac)));
    newColors[physicalToChain(next)] = _ring->Color(
        (uint8_t)(_ringR * frac),
        (uint8_t)(_ringG * frac),
        (uint8_t)(_ringB * frac));

    // Diff against current buffer; only show() if anything changed.
    bool changed = false;
    for (int i = 0; i < RING_LEDS; i++) {
        if (_ring->getPixelColor(i) != newColors[i]) {
            _ring->setPixelColor(i, newColors[i]);
            changed = true;
        }
    }
    if (changed) _ring->show();
}

int32_t RotaryEncoder::getPosition() {
    return _position;
}

void RotaryEncoder::setPosition(int32_t pos) {
    _position = pos;
    _rawPosition = pos;
    // Re-baseline the poll delta to the counter's current reading so the next
    // update() computes a zero delta from the new value.
    if (_usePcnt && _enc) {
        _lastPolledDetent = _countToDetent(_enc->getCount());
    }
    _lastRingPosition = INT32_MIN;  // force ring re-render on next update
}

void RotaryEncoder::resetPosition() {
    setPosition(0);
}

int RotaryEncoder::getDirection() {
    int dir = _lastDirection;
    _lastDirection = 0;
    return dir;
}

bool RotaryEncoder::wasRotatedCW() {
    if (_cwFlag) {
        _cwFlag = false;
        return true;
    }
    return false;
}

bool RotaryEncoder::wasRotatedCCW() {
    if (_ccwFlag) {
        _ccwFlag = false;
        return true;
    }
    return false;
}

bool RotaryEncoder::isPressed() {
    if (_pinSw < 0) return false;
    return digitalRead(_pinSw) == LOW;
}

bool RotaryEncoder::wasPressed() {
    if (_pressedFlag) {
        _pressedFlag = false;
        return true;
    }
    return false;
}

void RotaryEncoder::setAcceleration(bool enabled, int maxMultiplier) {
    _accelEnabled = enabled;
    _accelMaxMultiplier = max(1, maxMultiplier);
}

// --- Ring methods ---
// These only stage state; the actual render happens on the next update() so all
// NeoPixel writes stay on one task. INT32_MIN forces a repaint next update().

void RotaryEncoder::enableRing(bool enabled) {
    _ringEnabled = enabled;
    if (!enabled && _ring) {
        _ring->clear();
        _ring->show();
    }
    _lastRingPosition = INT32_MIN;
}

void RotaryEncoder::setRingColor(uint8_t r, uint8_t g, uint8_t b) {
    _ringR = r;
    _ringG = g;
    _ringB = b;
    _lastRingPosition = INT32_MIN;
}

void RotaryEncoder::setRingBrightness(uint8_t brightness) {
    if (_ring) {
        _ring->setBrightness(brightness);
        _lastRingPosition = INT32_MIN;
    }
}

void RotaryEncoder::setRingPosition(float pos) {
    if (pos < 0 || isnan(pos)) {
        _ringOverridePos = -1.0f;  // Re-couple to knob
    } else {
        _ringOverridePos = pos;
    }
    _lastRingPosition = INT32_MIN;
}
