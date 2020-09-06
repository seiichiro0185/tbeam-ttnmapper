## TTGO T-Beam TTNMapper Sketch

A Firmware for the TTGO T-Beam and Heltec Wireless Stick to use it as a standalone TTNMapper (-> https://ttnmapper.org)
Based on the Ardino Sketch at https://github.com/Bjoerns-TB/Lora-TTNMapper-T-Beam

- Converted to PlatformIO
- Use Second ESP32-Core for UI Thread (better button responsivness)
- Always show Satellites in view / in use
- Support button to switch through different send intervals

The Code needs the following Libraries (they are configured as dependencies in platformio.ini):

- LMIC-Arduino
- SimpleButton
- TinyGPSPlus
- U8g2

Please Insert your Network / Application Keys and Device ID from the TTN-Console into include/config.h before flashing, it will not work without. An example of config.h you find in /include/config.sample

The platformio.ini configures the Pins for all 3 boards depending on the ENV of your choice

For Heltec Wireless Stick (no integrated GPS) you have to solder the GPS to Pins RX 23 and TX 17. The 64x32 OLED is very limited to visualize things. The onboard LED is on as long a TTN packet is queued

## License

All Code in this Repository provided under a 3-Clause BSD License unless stated otherwise:

Copyright 2019 <Stefan Brand>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
