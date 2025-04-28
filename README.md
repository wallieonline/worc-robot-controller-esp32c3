# worc-robot-controller-esp8266
Firmware for the WORC Robot Controller based on the Espressif ESP32-C3 MCU and ESP-NOW protocol.
For more information about the project see:<br/>
(https://www.wallieonline.nl/blogs/esp-now-remote-control-mini-robots.html)

![WORC Robot Controller](https://img.youtube.com/vi/p5JXhyKQbI0/hqdefault.jpg)

# Finding your robot-controller MAC address
- Use readmac.ino on the robot-controller to retrieve the MAC address.
- Save the MAC address.
- Use the MAC address in the remote-controller project.

# Programming the ESP32 remote-controller
- Install the Arduino IDE software from the Arduino website.
- After starting the software go to preferences.
- Past the "Additional Boards Manager URLs" for ESP32. (https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json)
- Open the "Boards Manager" and install "esp32 by Espressive Systems version 2.0.14"
- Use version 2.0.14 other versions may not work!
- Use a USB data transfer kabel! Some cables are charge only!
- Put your ESP32 board in serial bootloader mode by keeping the Flash/BOOT button pressed when powering up.
- Install the driver for your ESP32 board if needed.
- Select the correct Board, Settings and port for your ESP32 board and click upload.