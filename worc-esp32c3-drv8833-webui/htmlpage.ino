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

String getHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>WORC Robot Controller</title>
  <style>
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      -webkit-touch-callout: none;
      -webkit-user-select: none;
      -khtml-user-select: none;
      -moz-user-select: none;
      -ms-user-select: none;
      user-select: none;
    }
    body {
      background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      justify-content: center;
      align-items: center;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      overflow-x: hidden;
      padding: 15px 0;
    }
    h1 {
      color: #eee;
      margin-bottom: 15px;
      font-size: 1.5rem;
      text-align: center;
    }
    .status {
      color: #0f0;
      font-size: 0.9rem;
      padding: 8px 16px;
      background: rgba(0, 255, 0, 0.1);
      border: 1px solid #0f0;
      border-radius: 20px;
    }
    .status.disconnected {
      color: #f00;
      background: rgba(255, 0, 0, 0.1);
      border-color: #f00;
    }
    .top-bar {
      display: flex;
      align-items: center;
      gap: 12px;
      margin-bottom: 18px;
      flex-wrap: wrap;
      justify-content: center;
    }
    .mode-tabs {
      display: flex;
      gap: 8px;
    }
    .mode-btn {
      padding: 8px 18px;
      background: rgba(0,0,0,0.3);
      color: #888;
      border: 2px solid #4a4a6a;
      border-radius: 20px;
      font-size: 0.85rem;
      font-weight: bold;
      cursor: pointer;
      font-family: inherit;
    }
    .mode-btn.active {
      color: #fff;
      border-color: #ff6b6b;
      background: rgba(255, 107, 107, 0.15);
      box-shadow: 0 0 12px rgba(255, 107, 107, 0.3);
    }
    .mode-panel {
      display: flex;
      align-items: center;
      justify-content: center;
      min-height: 250px;
    }
    .joystick-container {
      position: relative;
      width: 250px;
      height: 250px;
    }
    .joystick-base {
      position: absolute;
      width: 250px;
      height: 250px;
      background: radial-gradient(circle at 30% 30%, #3a3a5a, #1a1a2e);
      border-radius: 50%;
      border: 3px solid #4a4a6a;
      box-shadow: 0 10px 30px rgba(0,0,0,0.5), inset 0 2px 10px rgba(255,255,255,0.1);
    }
    .joystick-base::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 80px;
      height: 80px;
      transform: translate(-50%, -50%);
      border: 2px dashed #5a5a7a;
      border-radius: 50%;
      opacity: 0.5;
    }
    .joystick-base::after {
      content: '';
      position: absolute;
      top: 50%;
      left: 10%;
      right: 10%;
      height: 2px;
      background: linear-gradient(90deg, transparent, #5a5a7a, transparent);
      transform: translateY(-50%);
    }
    .joystick-handle {
      position: absolute;
      width: 80px;
      height: 80px;
      background: radial-gradient(circle at 30% 30%, #ff6b6b, #c92a2a);
      border-radius: 50%;
      border: 3px solid #fff;
      box-shadow: 0 5px 20px rgba(0,0,0,0.4), inset 0 -3px 10px rgba(0,0,0,0.2);
      cursor: pointer;
      touch-action: none;
      left: 85px;
      top: 85px;
      transition: transform 0.1s ease-out;
    }
    .joystick-handle:active {
      transform: scale(0.95);
      background: radial-gradient(circle at 30% 30%, #ff5252, #b91c1c);
    }
    .joystick-handle::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 30px;
      height: 30px;
      transform: translate(-50%, -50%);
      background: radial-gradient(circle at 30% 30%, rgba(255,255,255,0.4), transparent);
      border-radius: 50%;
    }
    .crosshair {
      position: absolute;
      top: 50%;
      left: 50%;
      width: 20px;
      height: 2px;
      background: #5a5a7a;
      transform: translate(-50%, -50%);
    }
    .crosshair::after {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 2px;
      height: 20px;
      background: #5a5a7a;
      transform: translate(-50%, -50%);
    }
    .labels {
      position: absolute;
      width: 100%;
      height: 100%;
      pointer-events: none;
    }
    .label {
      position: absolute;
      color: #888;
      font-size: 0.8rem;
      font-weight: bold;
    }
    .label-top { top: 10px; left: 50%; transform: translateX(-50%); }
    .label-bottom { bottom: 10px; left: 50%; transform: translateX(-50%); }
    .label-left { left: 10px; top: 50%; transform: translateY(-50%); }
    .label-right { right: 10px; top: 50%; transform: translateY(-50%); }
    /* Sliders (tank mode) */
    .sliders-row {
      display: flex;
      gap: 70px;
      align-items: center;
    }
    .slider {
      position: relative;
      background: radial-gradient(circle at 30% 30%, #3a3a5a, #1a1a2e);
      border: 3px solid #4a4a6a;
      box-shadow: 0 10px 30px rgba(0,0,0,0.5), inset 0 2px 10px rgba(255,255,255,0.1);
      touch-action: none;
    }
    .slider-v {
      width: 90px;
      height: 210px;
      border-radius: 45px;
    }
    .slider-h {
      width: 210px;
      height: 90px;
      border-radius: 45px;
    }
    .slider::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 40px;
      height: 40px;
      transform: translate(-50%, -50%);
      border: 2px dashed #5a5a7a;
      border-radius: 50%;
      opacity: 0.5;
    }
    .slider-handle {
      position: absolute;
      width: 70px;
      height: 70px;
      background: radial-gradient(circle at 30% 30%, #ff6b6b, #c92a2a);
      border-radius: 50%;
      border: 3px solid #fff;
      box-shadow: 0 5px 20px rgba(0,0,0,0.4), inset 0 -3px 10px rgba(0,0,0,0.2);
      cursor: pointer;
      touch-action: none;
      transition: transform 0.1s ease-out;
    }
    .slider-handle:active {
      transform: scale(0.95);
      background: radial-gradient(circle at 30% 30%, #ff5252, #b91c1c);
    }
    .slider-handle::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 26px;
      height: 26px;
      transform: translate(-50%, -50%);
      background: radial-gradient(circle at 30% 30%, rgba(255,255,255,0.4), transparent);
      border-radius: 50%;
    }
    .slider-v .slider-handle { left: 7px; top: 67px; }
    .slider-h .slider-handle { left: 67px; top: 7px; }
    .slider-tag {
      position: absolute;
      bottom: -26px;
      left: 50%;
      transform: translateX(-50%);
      color: #888;
      font-size: 0.75rem;
      font-weight: bold;
      pointer-events: none;
      white-space: nowrap;
    }
    .slider-end {
      position: absolute;
      color: #888;
      font-size: 0.7rem;
      font-weight: bold;
      pointer-events: none;
    }
    .slider-v .end-top { top: 12px; left: 50%; transform: translateX(-50%); }
    .slider-v .end-bottom { bottom: 12px; left: 50%; transform: translateX(-50%); }
    .slider-h .end-left { left: 12px; top: 50%; transform: translateY(-50%); }
    .slider-h .end-right { right: 12px; top: 50%; transform: translateY(-50%); }
    /* Settings panel */
    .settings {
      display: flex;
      gap: 8px;
      margin-top: 34px;
      flex-wrap: wrap;
      justify-content: center;
    }
    .toggle-btn {
      padding: 8px 14px;
      background: rgba(0,0,0,0.3);
      color: #888;
      border: 2px solid #4a4a6a;
      border-radius: 20px;
      font-size: 0.8rem;
      font-weight: bold;
      cursor: pointer;
      font-family: inherit;
    }
    .toggle-btn.on {
      color: #0f0;
      border-color: #0f0;
      background: rgba(0, 255, 0, 0.1);
      box-shadow: 0 0 10px rgba(0, 255, 0, 0.2);
    }
    .value-display {
      margin-top: 10px;
      padding: 10px 20px;
      background: rgba(0,0,0,0.3);
      border-radius: 10px;
      color: #0f0;
      font-family: 'Courier New', monospace;
      font-size: 0.9rem;
    }
    .telemetry {
      margin-top: 10px;
      padding: 8px 16px;
      background: rgba(0,0,0,0.3);
      border: 1px solid #4a4a6a;
      border-radius: 10px;
      color: #6ab0ff;
      font-family: 'Courier New', monospace;
      font-size: 0.8rem;
    }
    .footer {
      margin-top: 18px;
      color: #555;
      font-size: 0.75rem;
    }
    .footer a {
      color: inherit;
    }
  </style>
</head>
<body>
  <h1>🎮 WORC Robot Controller</h1>

  <div class="top-bar">
    <div class="status" id="status">Connecting...</div>
    <div class="mode-tabs">
      <button class="mode-btn active" data-mode="joystick">Joystick</button>
      <button class="mode-btn" data-mode="tank">Tank</button>
    </div>
  </div>

  <div class="mode-panel" id="panel-joystick">
    <div class="joystick-container">
      <div class="joystick-base">
        <div class="crosshair"></div>
        <div class="labels">
          <span class="label label-top">FORWARD</span>
          <span class="label label-bottom">REVERSE</span>
          <span class="label label-left">LEFT</span>
          <span class="label label-right">RIGHT</span>
        </div>
      </div>
      <div class="joystick-handle" id="joystick"></div>
    </div>
  </div>

  <div class="mode-panel" id="panel-tank" style="display:none">
    <div class="sliders-row">
      <div class="slider slider-v" id="sliderL">
        <span class="slider-end end-top">FWD</span>
        <span class="slider-end end-bottom">REV</span>
        <div class="slider-handle"></div>
        <span class="slider-tag">LEFT MOTOR</span>
      </div>
      <div class="slider slider-v" id="sliderR">
        <span class="slider-end end-top">FWD</span>
        <span class="slider-end end-bottom">REV</span>
        <div class="slider-handle"></div>
        <span class="slider-tag">RIGHT MOTOR</span>
      </div>
    </div>
  </div>

  <div class="settings">
    <button class="toggle-btn" id="btnInvertL">Invert Left: OFF</button>
    <button class="toggle-btn" id="btnInvertR">Invert Right: OFF</button>
    <button class="toggle-btn on" id="btnMixer">Mixer: ON</button>
  </div>

  <div class="value-display" id="values">Throttle: 0 | Steering: 0</div>
  <div class="telemetry" id="telemetry">Telemetry: --</div>

  <div class="footer">WORC Robot Controller<br><a href="https://github.com/wallieonline">© 2026 wallieonline</a></div>

  <script>
    const joystick = document.getElementById('joystick');
    const container = document.querySelector('.joystick-container');
    const statusDiv = document.getElementById('status');
    const valuesDiv = document.getElementById('values');
    const telemetryDiv = document.getElementById('telemetry');

    let isDragging = false;
    let joyTouchId = null;
    let centerX, centerY;
    const maxRadius = 85;
    const deadzone = 10;

    let throttle = 0;
    let steering = 0;
    let tankL = 0;
    let tankR = 0;
    let mode = 'joystick';
    let lastTlmTime = 0;
    let gotAnyTlm = false;

    // Settings mirrored from the device
    let cfgInvertL = false, cfgInvertR = false, cfgMixer = true;

    // WebSocket connection
    let ws;
    let reconnectInterval;

    function sendCommand() {
      if (!ws || ws.readyState !== WebSocket.OPEN) return;
      if (mode === 'tank') {
        ws.send(tankL + ',' + tankR);
      } else {
        ws.send(throttle + ',' + steering);
      }
    }

    function sendConfig() {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send('CFG,' + (cfgInvertL ? 1 : 0) + ',' + (cfgInvertR ? 1 : 0) + ',' + (cfgMixer ? 1 : 0));
      }
    }

    function updateToggleButtons() {
      const bL = document.getElementById('btnInvertL');
      const bR = document.getElementById('btnInvertR');
      const bM = document.getElementById('btnMixer');
      bL.textContent = 'Invert Left: ' + (cfgInvertL ? 'ON' : 'OFF');
      bR.textContent = 'Invert Right: ' + (cfgInvertR ? 'ON' : 'OFF');
      bM.textContent = 'Mixer: ' + (cfgMixer ? 'ON' : 'OFF');
      bL.classList.toggle('on', cfgInvertL);
      bR.classList.toggle('on', cfgInvertR);
      bM.classList.toggle('on', cfgMixer);
      applyMixerLayout();
    }

    document.getElementById('btnInvertL').addEventListener('click', function() {
      cfgInvertL = !cfgInvertL; updateToggleButtons(); sendConfig();
    });
    document.getElementById('btnInvertR').addEventListener('click', function() {
      cfgInvertR = !cfgInvertR; updateToggleButtons(); sendConfig();
    });
    document.getElementById('btnMixer').addEventListener('click', function() {
      cfgMixer = !cfgMixer; updateToggleButtons(); sendConfig();
    });

    // Mode switching
    document.querySelectorAll('.mode-btn').forEach(function(btn) {
      btn.addEventListener('click', function() {
        document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        mode = btn.dataset.mode;
        ['joystick','tank'].forEach(function(m) {
          document.getElementById('panel-' + m).style.display = (m === mode) ? 'flex' : 'none';
        });
        // Stop everything when switching modes
        isDragging = false; joyTouchId = null;
        throttle = 0; steering = 0; tankL = 0; tankR = 0;
        resetJoystick();
        resetSlider(sliderL); resetSlider(sliderR);
        updateValueDisplay();
        sendCommand();
      });
    });

    function updateValueDisplay() {
      if (mode === 'tank') {
        valuesDiv.textContent = cfgMixer
          ? `Throttle: ${tankL} | Steering: ${tankR}`
          : `Left: ${tankL} | Right: ${tankR}`;
      } else {
        valuesDiv.textContent = `Throttle: ${throttle} | Steering: ${steering}`;
      }
    }

    function connect() {
      clearInterval(reconnectInterval);

      ws = new WebSocket('ws://' + window.location.hostname + ':81/');

      ws.onopen = function() {
        console.log('WebSocket connected');
        statusDiv.textContent = 'Connected';
        statusDiv.classList.remove('disconnected');
        clearInterval(reconnectInterval);
        gotAnyTlm = false; // reset: only trust the watchdog once real telemetry arrives
        ws.send('CFG?'); // request stored settings
      };

      ws.onclose = function() {
        console.log('WebSocket disconnected');
        statusDiv.textContent = 'Disconnected - Reconnecting...';
        statusDiv.classList.add('disconnected');
        reconnectInterval = setInterval(connect, 2000);
      };

      ws.onerror = function(err) {
        console.error('WebSocket error:', err);
      };

      ws.onmessage = function(evt) {
        const msg = evt.data;
        if (msg.startsWith('TLM,')) {
          const p = msg.split(',');
          lastTlmTime = Date.now();
          gotAnyTlm = true; // firmware supports telemetry - the watchdog below may act on it
          const mixerTxt = cfgMixer ? 'on' : 'off';
          telemetryDiv.textContent =
            `Motor L: ${p[1]} | R: ${p[2]} | Thr: ${p[3]} | Str: ${p[4]} | mixer: ${mixerTxt} | RSSI: ${p[5]} dBm`;
        } else if (msg.startsWith('CFG,')) {
          const p = msg.split(',');
          cfgInvertL = p[1] === '1';
          cfgInvertR = p[2] === '1';
          cfgMixer = p[3] === '1';
          updateToggleButtons();
        }
      };
    }

    connect();

    // Resend the current value every 100 ms so the on-board timeout doesn't fire while holding the joystick still.
    setInterval(sendCommand, 100);

    // Watchdog: if telemetry (200 ms when connected) goes quiet, treat the link as dead and force a reconnect.
    const TELEMETRY_STALE_MS = 1000;
    setInterval(function() {
      if (gotAnyTlm && Date.now() - lastTlmTime > TELEMETRY_STALE_MS) {
        telemetryDiv.textContent = 'Telemetry: no data';
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.close(); // triggers onclose -> "Disconnected - Reconnecting..." + retry loop
        }
      }
    }, 500);

    function updateCenter() {
      const rect = container.getBoundingClientRect();
      centerX = rect.left + rect.width / 2;
      centerY = rect.top + rect.height / 2;
    }

    function resetJoystick() {
      joystick.style.left = '85px';
      joystick.style.top = '85px';
    }

    function handleStart(e) {
      if (isDragging) return; // Ignore a second finger while the first is steering
      if (e.touches) joyTouchId = e.changedTouches[0].identifier;
      isDragging = true;
      updateCenter();
      handleMove(e);
    }

    // Find the touch that is steering the joystick, so other fingers can't make it jump.
    function joyTouchFor(e) {
      if (joyTouchId === null) return e.touches[0] || null;
      for (let i = 0; i < e.touches.length; i++) {
        if (e.touches[i].identifier === joyTouchId) return e.touches[i];
      }
      return null;
    }

    function handleMove(e) {
      if (!isDragging) return;

      e.preventDefault();
      let clientX, clientY;
      if (!e.touches) {
        clientX = e.clientX;
        clientY = e.clientY;
      } else {
        const t = joyTouchFor(e);
        if (!t) return;
        clientX = t.clientX;
        clientY = t.clientY;
      }

      let deltaX = clientX - centerX;
      let deltaY = clientY - centerY;

      const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

      if (distance > maxRadius) {
        const ratio = maxRadius / distance;
        deltaX *= ratio;
        deltaY *= ratio;
      }

      if (distance < deadzone) {
        deltaX = 0;
        deltaY = 0;
      }

      joystick.style.left = (85 + deltaX) + 'px';
      joystick.style.top = (85 + deltaY) + 'px';

      throttle = Math.round(-deltaY / maxRadius * 255);
      steering = Math.round(deltaX / maxRadius * 255);

      updateValueDisplay();
      sendCommand();
    }

    function handleEnd(e) {
      if (!isDragging) return;
      if (e && e.changedTouches) {
        // Only release if the finger that lifted is the one steering the joystick
        let released = false;
        for (let i = 0; i < e.changedTouches.length; i++) {
          if (e.changedTouches[i].identifier === joyTouchId) { released = true; break; }
        }
        if (!released) return;
      }
      isDragging = false;
      joyTouchId = null;
      resetJoystick();
      throttle = 0;
      steering = 0;
      updateValueDisplay();
      sendCommand();
    }

    joystick.addEventListener('mousedown', handleStart);
    document.addEventListener('mousemove', handleMove);
    document.addEventListener('mouseup', handleEnd);

    joystick.addEventListener('touchstart', handleStart, {passive: false});
    document.addEventListener('touchmove', handleMove, {passive: false});
    document.addEventListener('touchend', handleEnd);
    document.addEventListener('touchcancel', handleEnd);

    // Spring-back slider, vertical/horizontal via setAxis(), tracks its own touch so two sliders can be dragged at once.
    function makeSlider(el, onValue) {
      const handle = el.querySelector('.slider-handle');
      const s = { el, handle, dragging: false, touchId: null, value: 0, axis: 'v' };

      function travel() {
        const size = s.axis === 'h' ? el.clientWidth : el.clientHeight;
        return size / 2 - 35; // handle radius
      }

      function setOffset(px) {
        const t = travel();
        px = Math.max(-t, Math.min(t, px));
        if (Math.abs(px) < 8) px = 0; // deadzone
        if (s.axis === 'h') {
          handle.style.left = (el.clientWidth / 2 - 35 + px) + 'px';
          handle.style.top = (el.clientHeight / 2 - 35) + 'px';
          s.value = t > 0 ? Math.round(px / t * 255) : 0;
        } else {
          handle.style.left = (el.clientWidth / 2 - 35) + 'px';
          handle.style.top = (el.clientHeight / 2 - 35 + px) + 'px';
          s.value = t > 0 ? Math.round(-px / t * 255) : 0;
        }
        onValue(s.value);
      }

      s.reset = function() { setOffset(0); };

      s.setAxis = function(axis) {
        if (s.axis === axis) return;
        s.axis = axis;
        el.classList.toggle('slider-v', axis === 'v');
        el.classList.toggle('slider-h', axis === 'h');
        const ends = el.querySelectorAll('.slider-end');
        if (ends[0]) ends[0].className = 'slider-end ' + (axis === 'h' ? 'end-left' : 'end-top');
        if (ends[1]) ends[1].className = 'slider-end ' + (axis === 'h' ? 'end-right' : 'end-bottom');
        s.dragging = false;
        s.touchId = null;
        s.reset();
      };

      function touchFor(e) {
        if (s.touchId === null) return e.touches[0] || null;
        for (let i = 0; i < e.touches.length; i++) {
          if (e.touches[i].identifier === s.touchId) return e.touches[i];
        }
        return null; // this slider's finger isn't part of this event
      }

      function move(e) {
        if (!s.dragging) return;
        let clientX, clientY;
        if (!e.touches) {
          clientX = e.clientX; clientY = e.clientY; // mouse event
        } else {
          const t = touchFor(e);
          if (!t) return;
          clientX = t.clientX; clientY = t.clientY;
        }
        e.preventDefault();
        const r = el.getBoundingClientRect();
        setOffset(s.axis === 'h'
          ? clientX - (r.left + r.width / 2)
          : clientY - (r.top + r.height / 2));
      }
      s.move = move;

      function start(e) {
        if (e.touches) {
          const t = e.changedTouches[0];
          s.touchId = t.identifier;
          s.dragging = true;
          e.preventDefault();
          const r = el.getBoundingClientRect();
          setOffset(s.axis === 'h'
            ? t.clientX - (r.left + r.width / 2)
            : t.clientY - (r.top + r.height / 2));
        } else {
          s.dragging = true;
          move(e);
        }
      }

      function end(e) {
        if (!s.dragging) return;
        if (e && e.changedTouches) {
          // Only release if the finger that lifted is the one driving this slider
          let released = false;
          for (let i = 0; i < e.changedTouches.length; i++) {
            if (e.changedTouches[i].identifier === s.touchId) { released = true; break; }
          }
          if (!released) return;
        }
        s.dragging = false;
        s.touchId = null;
        s.reset();
      }
      s.end = end;

      el.addEventListener('mousedown', start);
      el.addEventListener('touchstart', start, {passive: false});
      return s;
    }

    function resetSlider(s) { s.dragging = false; s.touchId = null; s.reset(); }

    const sliderL = makeSlider(document.getElementById('sliderL'), v => { tankL = v; updateValueDisplay(); });
    const sliderR = makeSlider(document.getElementById('sliderR'), v => { tankR = v; updateValueDisplay(); });

    function setSliderLabels(el, first, second, tag) {
      const ends = el.querySelectorAll('.slider-end');
      if (ends[0]) ends[0].textContent = first;
      if (ends[1]) ends[1].textContent = second;
      const t = el.querySelector('.slider-tag');
      if (t) t.textContent = tag;
    }

    // Mixer ON: left slider = vertical throttle, right slider = horizontal steering; OFF: two vertical motor sliders.
    function applyMixerLayout() {
      const elL = document.getElementById('sliderL');
      const elR = document.getElementById('sliderR');
      if (cfgMixer) {
        sliderR.setAxis('h');
        setSliderLabels(elR, 'LEFT', 'RIGHT', 'STEERING');
        setSliderLabels(elL, 'FWD', 'REV', 'THROTTLE');
      } else {
        sliderR.setAxis('v');
        setSliderLabels(elR, 'FWD', 'REV', 'RIGHT MOTOR');
        setSliderLabels(elL, 'FWD', 'REV', 'LEFT MOTOR');
      }
    }
    applyMixerLayout();

    const allSliders = [sliderL, sliderR];
    document.addEventListener('mousemove', e => allSliders.forEach(s => s.move(e)));
    document.addEventListener('touchmove', e => {
      let any = false;
      allSliders.forEach(s => { if (s.dragging) any = true; s.move(e); });
      if (any || isDragging) e.preventDefault();
    }, {passive: false});
    document.addEventListener('mouseup', () => allSliders.forEach(s => s.end()));
    document.addEventListener('touchend', e => allSliders.forEach(s => s.end(e)));
    document.addEventListener('touchcancel', e => allSliders.forEach(s => s.end(e)));

    window.addEventListener('resize', updateCenter);
    window.addEventListener('scroll', updateCenter);
  </script>
</body>
</html>
)rawliteral";
}