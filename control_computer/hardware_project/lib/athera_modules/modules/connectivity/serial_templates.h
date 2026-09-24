/**
 * @file serial_templates.h
 * @brief AtechSerial — USB-Serial transport mirroring the AtechWiFi public API
 *
 * Header-only serial transport for Atech IoT projects. JSON-over-UART,
 * newline-delimited, on the primary `Serial` (USB-CDC). Same envelopes as
 * the WebSocket gateway — clients (Web Serial in browser, future Python
 * package) speak the same protocol over either transport.
 *
 * Protocol:
 *   Outbound: {"type":"event","payload":{"event_type":"...","key":"...","value":...}}\n
 *   Inbound:  {"action":"...","value":"..."}\n  — dispatched to onMessage callback
 *
 * Shared UART with module logs: maintain() ignores any line that does not
 * parse as JSON, and only fires the callback when an `action` field is
 * present. Module log output prefixed with `[...]` is silently discarded.
 */

#ifndef ATHERA_SERIAL_TEMPLATES_H
#define ATHERA_SERIAL_TEMPLATES_H

#include <Arduino.h>
#include <ArduinoJson.h>

class AtechSerial {
public:
    AtechSerial(unsigned long baud = 115200)
        : _baud(baud)
        , _connected(false)
        , _rxLen(0)
        , _eventsPosted(0)
        , _messagesReceived(0)
    {
    }

    bool connect() {
        if (!_connected) {
            // Bump the USB-CDC RX ring buffer before begin(). Default is 256 bytes,
            // which overflows when an action handler blocks (e.g. play_pcm calling
            // i2s_write for 30–60 ms while the host streams the next chunk). 8 KB
            // is comfortable for paced PCM streaming with ~3 KB chunks.
            Serial.setRxBufferSize(8192);
            Serial.begin(_baud);
            // TX must be non-blocking even on this transport. If the host pauses
            // reading (debugger detaches, monitor closes, app crashes), default
            // USB-CDC behavior is to block Serial.print* on a full buffer — which
            // would freeze the firmware. Drop excess bytes instead; protocol JSON
            // is small and the host normally drains far faster than we emit.
            Serial.setTxTimeoutMs(0);
            _connected = true;
            Serial.println("[Atech Serial] Ready");
        }
        return true;
    }

    void maintain() {
        while (Serial.available() > 0) {
            int b = Serial.read();
            if (b < 0) break;
            char c = (char)b;
            if (c == '\n' || c == '\r') {
                if (_rxLen > 0) {
                    _rxBuf[_rxLen] = '\0';
                    _handleLine(_rxBuf);
                    _rxLen = 0;
                }
            } else if (_rxLen < sizeof(_rxBuf) - 1) {
                _rxBuf[_rxLen++] = c;
            } else {
                _rxLen = 0;
            }
        }
    }

    bool isConnected() { return _connected; }
    bool isWebSocketConnected() { return _connected; }

    typedef void (*MessageCallback)(const char* action, const char* value);
    void onMessage(MessageCallback cb) { _messageCallback = cb; }

    bool postSensorEvent(const char* key, float value, const char* moduleType = "") {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "sensor";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        doc["payload"]["source"] = moduleType;
        return _sendEvent(doc);
    }

    bool postSensorEventInt(const char* key, int value, const char* moduleType = "") {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "sensor";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        doc["payload"]["source"] = moduleType;
        return _sendEvent(doc);
    }

    // String-valued sensor events — for multi-value readings packed as a string
    // (e.g. robot arm joint angles "0.5,0.3,-0.2,...") or structured JSON snippets.
    bool postSensorEventStr(const char* key, const char* value, const char* moduleType = "") {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "sensor";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        doc["payload"]["source"] = moduleType;
        return _sendEvent(doc);
    }

    bool postButtonEvent(const char* key, int value) {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "button";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        return _sendEvent(doc);
    }

    bool postStateEvent(const char* key, const char* value) {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "state";
        doc["payload"]["key"] = key;
        doc["payload"]["value"] = value;
        return _sendEvent(doc);
    }

    bool postLogEvent(const char* message) {
        JsonDocument doc;
        doc["type"] = "event";
        doc["payload"]["event_type"] = "log";
        doc["payload"]["key"] = "log";
        doc["payload"]["value"] = message;
        return _sendEvent(doc);
    }

    // ── String overloads ──────────────────────────────────────────────
    // Accept Arduino String / StringSumHelper so the natural codegen idiom
    // `postLogEvent("x=" + String(value))` compiles. (`const char* + String`
    // yields a StringSumHelper, which has no implicit conversion to const
    // char*.) String literals still bind to the const char* overloads above;
    // these forward via .c_str().
    bool postLogEvent(const String& message) { return postLogEvent(message.c_str()); }
    bool postStateEvent(const char* key, const String& value) { return postStateEvent(key, value.c_str()); }
    bool postSensorEventStr(const char* key, const String& value, const char* moduleType = "") {
        return postSensorEventStr(key, value.c_str(), moduleType);
    }

    int getEventsPosted() { return _eventsPosted; }
    int getMessagesReceived() { return _messagesReceived; }

    String getStats() {
        JsonDocument doc;
        doc["events_posted"] = _eventsPosted;
        doc["messages_received"] = _messagesReceived;
        doc["transport"] = "serial";
        String out;
        serializeJson(doc, out);
        return out;
    }

private:
    unsigned long _baud;
    bool _connected;

    // 4096 lets a single line carry ~3 KB of base64 payload (≈ 2.2 KB binary,
    // ~1100 int16 samples ≈ 70 ms of 16 kHz mono PCM) — large enough that
    // play_pcm chunks for MP3 streaming fit comfortably with JSON overhead.
    static const size_t RX_BUF_SIZE = 4096;
    char _rxBuf[RX_BUF_SIZE];
    size_t _rxLen;

    int _eventsPosted;
    int _messagesReceived;
    MessageCallback _messageCallback = nullptr;

    void _handleLine(const char* line) {
        const char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p != '{') return;

        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, line);
        if (err) return;

        _messagesReceived++;

        if (!_messageCallback) return;

        // Scalars pass through as text; objects and arrays get serialized back
        // to a JSON substring so structured actions (e.g. set_color
        // {"r":255,"g":0,"b":0}) survive intact to the handler.
        const char* action = doc["action"] | doc["type"] | "";
        JsonVariantConst v = doc["value"];
        if (v.isNull()) v = doc["message"];
        if (v.isNull()) v = doc["payload"]["message"];
        String valueStr;
        if (v.is<const char*>()) {
            const char* s = v.as<const char*>();
            if (s) valueStr = s;
        } else if (v.is<JsonObjectConst>() || v.is<JsonArrayConst>()) {
            serializeJson(v, valueStr);
        } else if (!v.isNull()) {
            valueStr = v.as<String>();
        }
        if (action[0] != '\0') {
            _messageCallback(action, valueStr.c_str());
        }
    }

    bool _sendEvent(JsonDocument& doc) {
        if (!_connected) return false;
        serializeJson(doc, Serial);
        Serial.println();
        _eventsPosted++;
        return true;
    }
};

#endif // ATHERA_SERIAL_TEMPLATES_H
