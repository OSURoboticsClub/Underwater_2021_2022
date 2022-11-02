# Notes for Arduino ESP32
This folder contains all of the code related to the ESP32 controller that currently manages:
1. Motor control from I2C from Jetson
2. (planned) Sensors

## TODO
1. Add multiple motor control
2. Program the other ESCs
3. Move over to PlatformIO instead of Arduino IDE


## Required Libraries
### Board Definition Installation
1. Arduino by default doesn't support flashing the ESP32. Please add the following to the board manager urls in your *File -> Preferences*:  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Then go to *Tools -> Boards -> Boards Manager* then search for `esp32` and install the latest version.
3. Go back to *Tools -> Boards -> ESP32 Arduino* submenu and select the *ESP32 Wrover Kit (all versions)*. Once selected make sure to select the right COM port (if it's plugged in) and leave the rest of the options the same.


**Recommended Install of Libraries** I highly recommend to go into your Arduino/libraries folder and git clone them into there since David will probably make a lot of changes to the ESC_ESP32 library and it's easier to `git pull origin main` than to use the *Download Zip* option.
1. [ESP32 Servo](https://github.com/jkb-git/ESP32Servo) library allows for easier control of ESC with the writeMicroseconds pulse command
2. [ESC_ESP32](https://github.com/smerkousdavid/ESC_ESP32) allows for easy ESC control with arming/calibration and speed/direction control. (**NOTE:** you have to use smerkousdavid's fork for this to work as the original repo is missing a lot of features/not final)



## Programming an ESC
There is a current problem with the way the ESCs are programmed from factory! They are programmed to be in forward mode only.

### Forward Mode
To control an ESC in forward mode a pulse width of 1000us is speed 0% and 2000us is 100%.

### Forward/Reverse Mode
To control an ESC in forward/reverse mode the new mapping range is actually speed 0% (neutral) is at 1500us. Then +-500us is the new forward reverse range. So 1000us is -100% and 2000us is 100%.


### Programming for Forward/Reverse
Flash the ESP32 with the following code then follow the steps below.
```arduino
#include <ESP32Servo.h>
#include "ESC.h"

#define ESC_PIN 25 // CHANGE THIS!

// vv ASSUMING THE ESC IS CURRENTLY IN FORWARD MODE ONLY vv
#define ESC_MAX_PULSE 2000
#define ESC_NEUTRAL_PULSE 1000
ESC esc(ESC_PIN, ESC::MODE_FORWARD_ONLY); // second arg doesn't really matter in this program since we're writing pulses directly
// ^^^^^^^^^^^^^^^

void setup() {
  Serial.begin(9600);

  esc.attach(ESC_PIN); // note we don't use arm for this one since we want 2000 msec pulses immediately
  esc.writePulse(ESC_MAX_PULSE);
} 
 
void loop() { 
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '0') {
      Serial.println("ZERO");
      esc.writePulse(ESC_NEUTRAL_PULSE);
    } else if (c == '1') {
      Serial.println("FULL");
      esc.writePulse(ESC_MAX_PULSE);
    }
  }
} 
```

1. Plug in your ESC and wait for 2 beeps (depending on model) then wait for 5 seconds until you hear a fun tone (again read your ESC datasheet).
  2. Then you'll hear 5 different groups of tones on repeat. First a `beep pause`, but wait for the `beep beep pause` then immediately send `0`.
  3. Then you will hear another set of tones that will repeat. First a `short beep` and then `two short beeps`. Once it reaches two `short beeps` immediately send `1.`
  4. Wait for a set tone (should sound like a jingle) to finish indicating you have set the forward/reverse mode on
  5. Unplug your ESC
  6. That's it your ESC is now enabled for forward/reverse when you plug it in