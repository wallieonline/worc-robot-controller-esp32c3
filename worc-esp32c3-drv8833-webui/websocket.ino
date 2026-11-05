/*
 * WORC Robot Controller - ESP32-C3 WiFi Robot Controller
 * Copyright (C) 2026 wallieonline (https://github.com/wallieonline)
 *
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * Additional terms under section 7 of the GPL apply (attribution and
 * naming, see the NOTICE file). This copyright notice and the
 * attribution may not be removed.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

// RX buffer for WebSocket frames; also reset when a new client is accepted.
static uint8_t buffer[256];
static size_t bufLen = 0;

// Base64 encoding for WebSocket handshake
const char BASE64_CHARS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

class MinimalSHA1 {
public:
  uint8_t digest[20];
  void init();
  void update(const uint8_t* data, size_t len);
  void final(uint8_t hash[20]);
private:
  uint32_t h[5];
  uint64_t total_len;
  uint8_t buffer[64];
  size_t buffer_len;
  void processBlock(const uint8_t* block);
};

void MinimalSHA1::init() {
  memset(digest, 0, 20);
  h[0] = 0x67452301;
  h[1] = 0xEFCDAB89;
  h[2] = 0x98BADCFE;
  h[3] = 0x10325476;
  h[4] = 0xC3D2E1F0;
  total_len = 0;
  buffer_len = 0;
}

void MinimalSHA1::processBlock(const uint8_t* block) {
  uint32_t w[80];
  for (int i = 0; i < 16; i++) {
    w[i] = ((uint32_t)block[i*4] << 24) | ((uint32_t)block[i*4+1] << 16) | ((uint32_t)block[i*4+2] << 8) | block[i*4+3];
  }
  for (int i = 16; i < 80; i++) {
    w[i] = ((w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16]) << 1) | ((w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16]) >> 31);
  }
  uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
  for (int i = 0; i < 80; i++) {
    uint32_t f, k;
    if (i < 20) { f = (b & c) | ((~b) & d); k = 0x5A827999; }
    else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
    else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
    else { f = b ^ c ^ d; k = 0xCA62C1D6; }
    uint32_t temp = ((a << 5) | (a >> 27)) + f + e + k + w[i];
    e = d; d = c; c = ((b << 30) | (b >> 2)); b = a; a = temp;
  }
  h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}

void MinimalSHA1::update(const uint8_t* data, size_t len) {
  total_len += len * 8;
  while (len > 0) {
    size_t copy = min(len, 64 - buffer_len);
    memcpy(buffer + buffer_len, data, copy);
    buffer_len += copy;
    data += copy;
    len -= copy;
    if (buffer_len == 64) {
      processBlock(buffer);
      buffer_len = 0;
    }
  }
}

void MinimalSHA1::final(uint8_t hash[20]) {
  buffer[buffer_len++] = 0x80;
  if (buffer_len > 56) {
    memset(buffer + buffer_len, 0, 64 - buffer_len);
    processBlock(buffer);
    buffer_len = 0;
  }
  memset(buffer + buffer_len, 0, 56 - buffer_len);
  buffer[56] = (total_len >> 56) & 0xFF;
  buffer[57] = (total_len >> 48) & 0xFF;
  buffer[58] = (total_len >> 40) & 0xFF;
  buffer[59] = (total_len >> 32) & 0xFF;
  buffer[60] = (total_len >> 24) & 0xFF;
  buffer[61] = (total_len >> 16) & 0xFF;
  buffer[62] = (total_len >> 8) & 0xFF;
  buffer[63] = total_len & 0xFF;
  processBlock(buffer);
  for (int i = 0; i < 5; i++) {
    hash[i*4] = (h[i] >> 24) & 0xFF;
    hash[i*4+1] = (h[i] >> 16) & 0xFF;
    hash[i*4+2] = (h[i] >> 8) & 0xFF;
    hash[i*4+3] = h[i] & 0xFF;
  }
}

String base64Encode(const uint8_t* data, size_t len) {
  String result;
  for (size_t i = 0; i < len; i += 3) {
    uint32_t val = data[i] << 16;
    if (i + 1 < len) val |= data[i+1] << 8;
    if (i + 2 < len) val |= data[i+2];
    result += BASE64_CHARS[(val >> 18) & 0x3F];
    result += BASE64_CHARS[(val >> 12) & 0x3F];
    result += (i + 1 < len) ? BASE64_CHARS[(val >> 6) & 0x3F] : '=';
    result += (i + 2 < len) ? BASE64_CHARS[val & 0x3F] : '=';
  }
  return result;
}

// Parse one frame; sets `consumed` (0 if incomplete), returns false on a close frame.
bool parseWebSocketFrame(uint8_t* data, size_t len, String& message, size_t& consumed) {
  consumed = 0;
  message = "";
  if (len < 2) return false;
  uint8_t opcode = data[0] & 0x0F;
  bool masked = (data[1] & 0x80) != 0;
  uint64_t payloadLen = data[1] & 0x7F;
  size_t headerLen = 2;
  if (payloadLen == 126) {
    if (len < 4) return false;
    payloadLen = (data[2] << 8) | data[3];
    headerLen = 4;
  } else if (payloadLen == 127) {
    if (len < 10) return false;
    payloadLen = 0;
    for (int i = 0; i < 8; i++) payloadLen = (payloadLen << 8) | data[2+i];
    headerLen = 10;
  }
  uint8_t maskKey[4] = {0};
  if (masked) {
    if (len < headerLen + 4) return false;
    memcpy(maskKey, data + headerLen, 4);
    headerLen += 4;
  }
  if (len < headerLen + payloadLen) return false; // Incomplete frame, wait for more data
  consumed = headerLen + payloadLen;
  if (opcode == 0x08) return false; // Close frame - caller should close connection
  if (opcode != 0x01 && opcode != 0x02) return true; // Control frame (e.g. ping/pong), nothing to do
  for (uint64_t i = 0; i < payloadLen && i < 100; i++) {
    uint8_t byte = data[headerLen + i];
    if (masked) byte ^= maskKey[i % 4];
    message += (char)byte;
  }
  return true;
}

// Build WebSocket text frame
void buildWebSocketFrame(const String& message, uint8_t* frame, size_t& frameLen) {
  frame[0] = 0x81; // FIN=1, opcode=text
  size_t msgLen = message.length();
  if (msgLen < 126) {
    frame[1] = msgLen;
    frameLen = 2;
  } else if (msgLen < 65536) {
    frame[1] = 126;
    frame[2] = (msgLen >> 8) & 0xFF;
    frame[3] = msgLen & 0xFF;
    frameLen = 4;
  } else {
    frame[1] = 127;
    for (int i = 0; i < 8; i++) frame[2+i] = (msgLen >> (56-i*8)) & 0xFF;
    frameLen = 10;
  }
  memcpy(frame + frameLen, message.c_str(), msgLen);
  frameLen += msgLen;
}

// Handle WebSocket handshake
bool handleWebSocketHandshake(String& request) {
  int keyStart = request.indexOf("Sec-WebSocket-Key: ");
  if (keyStart < 0) return false;
  keyStart += 19;
  int keyEnd = request.indexOf("\r\n", keyStart);
  if (keyEnd < 0) return false;
  String key = request.substring(keyStart, keyEnd);
  key.trim();
  // Compute accept key: base64(sha1(key + magic))
  String combined = key + WS_MAGIC;
  MinimalSHA1 sha1;
  sha1.init();
  sha1.update((const uint8_t*)combined.c_str(), combined.length());
  uint8_t hash[20];
  sha1.final(hash);
  String acceptKey = base64Encode(hash, 20);
  String response = "HTTP/1.1 101 Switching Protocols\r\n";
  response += "Upgrade: websocket\r\n";
  response += "Connection: Upgrade\r\n";
  response += "Sec-WebSocket-Accept: " + acceptKey + "\r\n";
  response += "\r\n";
  wsClient.print(response);
  return true;
}

// Dispatch one WebSocket text message: a settings command or "throttle,steering".
void handleWsMessage(const String& message) {
  if (message.startsWith("CFG?")) {
    // Client requests the current settings
    sendWebSocketMessage("CFG," + String(gInvertLeft ? 1 : 0) + "," + String(gInvertRight ? 1 : 0) + "," + String(gMixerOn ? 1 : 0));
  } else if (message.startsWith("CFG,")) {
    // Save settings: "CFG,invertLeft,invertRight,mixerOn"
    int c1 = message.indexOf(',', 4);
    int c2 = message.indexOf(',', c1 + 1);
    if (c1 > 4 && c2 > c1) {
      gInvertLeft  = message.substring(4, c1).toInt() != 0;
      gInvertRight = message.substring(c1 + 1, c2).toInt() != 0;
      gMixerOn     = message.substring(c2 + 1).toInt() != 0;
      saveSettings();
      sendWebSocketMessage("CFG," + String(gInvertLeft ? 1 : 0) + "," + String(gInvertRight ? 1 : 0) + "," + String(gMixerOn ? 1 : 0));
    }
  } else {
    // Parse joystick data: "throttle,steering"
    int commaIndex = message.indexOf(',');
    if (commaIndex > 0) {
      gThrottle = constrain(message.substring(0, commaIndex).toInt(), -255, 255);
      gSteering = constrain(message.substring(commaIndex + 1).toInt(), -255, 255);
      lastCommandTime = millis();
    }
  }
}

// Parse incoming WebSocket data
void processWebSocketData() {
  while (wsClient.available() && bufLen < sizeof(buffer)) {
    buffer[bufLen++] = wsClient.read();
  }
  // Dispatch every complete frame in order; newest joystick value wins anyway and CFG messages are never dropped.
  size_t offset = 0;
  while (offset < bufLen) {
    String message;
    size_t consumed = 0;
    bool ok = parseWebSocketFrame(buffer + offset, bufLen - offset, message, consumed);
    if (consumed == 0) break; // Frame not fully received yet, wait for more data
    offset += consumed;
    if (!ok) {
      // Close frame - stop the connection and drop anything still buffered
      wsClient.stop();
      bufLen = 0;
      break;
    }
    if (message.length() > 0) {
      handleWsMessage(message);
    }
  }
  // Keep any trailing partial frame for the next read instead of dropping it
  size_t remaining = (bufLen > offset) ? bufLen - offset : 0;
  if (offset > 0 && remaining > 0) {
    memmove(buffer, buffer + offset, remaining);
  }
  bufLen = remaining;
  if (bufLen >= sizeof(buffer)) bufLen = 0; // Overflow protection
}

// Check WebSocket connection
void handleWebSocket() {
  static uint32_t acceptTime = 0;
  // Accept new connections (a dead client gets replaced; mark it closed first)
  if (!wsClient.connected()) {
    if (wsConnected) {
      wsConnected = false;
      Serial.println("WebSocket disconnected");
    }
    wsClient = wsServer.accept();
    if (wsClient) {
      Serial.println("WebSocket client connecting...");
      wsConnected = false;
      acceptTime = millis();
      bufLen = 0; // Drop any frame bytes left behind by the previous client
    }
  }
  // Handle handshake
  if (wsClient && wsClient.connected() && !wsConnected) {
    if (wsClient.available()) {
      // Read headers with a hard deadline; a slow client must not stall loop() and the motor failsafe.
      String request;
      request.reserve(512);
      uint32_t deadline = millis() + 500;
      bool complete = false;
      while (wsClient.connected() && (int32_t)(millis() - deadline) < 0) {
        while (wsClient.available()) {
          request += (char)wsClient.read();
          if (request.endsWith("\r\n\r\n") || request.endsWith("\n\n")) {
            complete = true;
            break;
          }
        }
        if (complete || request.length() > 1024) break; // Done, or header too large
        delay(1); // Yield to WiFi/system tasks while waiting for more bytes
      }
      if (!complete) {
        wsClient.stop();
        return;
      }
      if (request.indexOf("Upgrade: websocket") >= 0) {
        if (handleWebSocketHandshake(request)) {
          wsConnected = true;
          wsClient.setNoDelay(true); // Send small frames immediately instead of waiting for Nagle batching
          Serial.println("WebSocket connected");
        } else {
          wsClient.stop();
        }
      } else {
        wsClient.stop();
      }
    } else if (millis() - acceptTime > 2000) {
      wsClient.stop(); // Idle pre-handshake connection must not block new clients forever
    }
  }
  // Handle WebSocket frames
  if (wsConnected && wsClient && wsClient.connected()) {
    if (wsClient.available()) {
      processWebSocketData();
    }
  } else if (wsConnected) {
    wsConnected = false;
    Serial.println("WebSocket disconnected");
  }
}

// Send WebSocket message
void sendWebSocketMessage(const String& msg) {
  if (wsConnected && wsClient && wsClient.connected() && msg.length() <= 200) {
    uint8_t frame[216]; // 10-byte max header + 200 payload
    size_t frameLen;
    buildWebSocketFrame(msg, frame, frameLen);
    wsClient.write(frame, frameLen);
  }
}