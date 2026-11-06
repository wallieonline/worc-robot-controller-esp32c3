# worc-robot-controller-esp32c3

Firmware for the WORC Robot Controller based on the Espressif ESP32-C3 MCU.

For more information about the project see:<br/>
https://wallieonline.nl/blogs/worc-robot-controller-en.html<br/>
https://wallieonline.nl/blogs/esp-now-remote-control-mini-robots.html

## Control options

The robot controller is available with different control interfaces. Pick the firmware variant that matches your setup:

- **ESP-NOW** (`worc-esp32c3-*-espnow`) — wireless remote using the WORC remote controller.
- **WebUI** (`worc-esp32c3-drv8833-webui`) — control the robot from a browser, no remote needed.
- **RC receiver** — connect a standard RC receiver using one of these protocols:
  - `worc-esp32c3-*-cppm` — CPPM (PPM sum signal) *(planned for a future release)*
  - `worc-esp32c3-*-sbus` — FrSky/Futaba SBUS *(planned for a future release)*
  - `worc-esp32c3-*-ibus` — FlySky iBUS *(planned for a future release)*
  - `worc-esp32c3-*-dsmx` — Spektrum DSMX *(planned for a future release)*
  - `worc-esp32c3-*-srxl` — Multiplex/Jeti SRXL *(planned for a future release)*
  - `worc-esp32c3-*-sumd` — Graupner SUMD *(planned for a future release)*
  - `worc-esp32c3-*-crsf` — TBS Crossfire / ELRS *(planned for a future release)*
  - `worc-esp32c3-*-pwm` — classic PWM servo signals (4 channels) *(planned for a future release)*

Each protocol variant exists for both motor drivers: `drv8833` and `tb6612fng`.

## Building instructions

For the worc-esp32c3-drv8833 version:<br/>
https://www.youtube.com/watch?v=jnXTLoNIQ_A

![WORC Robot Controller](https://img.youtube.com/vi/jnXTLoNIQ_A/hqdefault.jpg)

For the worc-esp32c3-tb6612fng version:<br/>
https://www.youtube.com/watch?v=p5JXhyKQbI0

![WORC Robot Controller](https://img.youtube.com/vi/p5JXhyKQbI0/hqdefault.jpg)

## Connecting the remote-controller to the robot-controller (ESP-NOW version)

No code changes or MAC addresses needed — the controllers find each other automatically.

1. Turn on the **robot** first. Its LED starts blinking: it is searching for a remote.
2. Within 30 seconds, turn on the **remote**. Its LED also blinks while searching.
3. When they find each other, the LEDs stay on — done! The remote now controls the robot.

Want to use a different remote? Turn both off and on again. Only turn on one remote at a time while connecting.

## Using the web interface (webui version)

1. Power the robot. It creates a WiFi access point named `WORC-XXXXXX` (the X's are the chip ID).
2. Connect your phone or computer to that WiFi network (password: `worc1234`).
3. Open a browser and go to `http://192.168.4.1`.
4. Drive the robot with the on-screen joystick.

The web page also has settings for inverting each motor and switching the throttle/steering mixer on or off. Settings are stored in EEPROM.

## Connecting an RC receiver

For the serial protocols (CPPM, SBUS, iBUS, DSMX, SRXL, SUMD, CRSF):

- Connect the receiver signal wire to **GPIO 4** and connect GND.
- Power the receiver according to its own specs (3.3V/5V from the board, or directly from the battery).
- Channel order is AETR: throttle on **ELE** (channel 2), steering on **AIL** (channel 1).

## Failsafe

All firmware variants have the same built-in safety behavior:

- If no valid control signal is received for **0.5 seconds**, both motors stop automatically.
- While the link is lost, the onboard LED **blinks**; when the link is healthy it stays on.
- This applies to every control method: a lost remote (ESP-NOW), a disconnected browser (WebUI), or a receiver failsafe / lost signal (RC protocols).

## Flashing with the webflasher (no installation)

The easiest way to install the firmware, no Arduino IDE needed:

1. Open the webflasher in **Chrome or Edge** (WebSerial is required): https://wallieonline.nl/flasher.html
2. Choose your motor driver (**DRV8833** or **TB6612FNG**) and the firmware variant for your control method.
3. Connect your ESP32 board via USB (a data cable, some cables are charge only!).
4. Click the **Flash** button under your chosen firmware.
5. In the popup from your browser, select the USB port and click **Connect**, then click **Install**. Keep the cable connected and this page open until it reaches 100% — done!

More firmware versions will be added to the webflasher soon.

*Tip: if flashing does not start, your board may need to be put in bootloader mode manually. Keep the Flash/BOOT button pressed when powering up the board and try again.*

## Programming the ESP32-C3 with the Arduino IDE

- Install the Arduino IDE software from the Arduino website.
- After starting the software go to Preferences.
- Paste the "Additional Boards Manager URLs" for ESP32: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
- Open the "Boards Manager" and install "esp32 by Espressif Systems version 2.0.14".
- Use version 2.0.14 — other versions may not work!
- Use a USB data transfer cable! Some cables are charge only!
- Put your ESP32 board in serial bootloader mode by keeping the Flash/BOOT button pressed when powering up.
- Install the driver for your ESP32 board if needed.
- Select the correct Board, Settings and port for your ESP32 board and click upload.

## License and attribution

This program is free software licensed under GPLv3 only (GPL-3.0-only, see [LICENSE](LICENSE)), with additional terms under Section 7 of the GPL.
The full text of these terms is in [NOTICE](NOTICE).
In short:

1. **Attribution:** if you distribute this software or a modified version, the credit "WORC Robot Controller — © 2026 wallieonline" with a link to https://github.com/wallieonline must be visibly included in the user interface or, if there is none, in the documentation and the source code headers.
   Do not remove or obscure the original copyright notices.
2. **Naming:** modified versions must be clearly marked as modified and use a distinct product name.
   The name "WORC" may not be used for modified versions.

These terms apply in addition to all GPLv3 conditions, including the requirement to publish your corresponding source code.