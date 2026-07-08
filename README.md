# LinkX Gesture Robot — Phone-Tilt Proportional Differential Drive

## Objective
Drive the rover by tilting your phone — front/back tilt controls throttle, left/right tilt controls steering — with true proportional (variable speed) differential-drive mixing, not just on/off commands.

## Design Note (read this first)
The original brief mentioned a hand-worn MPU6050, but a **phone's own built-in orientation sensors** (read via the browser's `DeviceOrientation` API) give the same "tilt to drive" experience with **zero extra wiring** — you just open the dashboard and tilt your phone. This build uses that approach. If you specifically want a wearable MPU6050 instead, see "Alternative" below.

## Real-World Applications
Accessibility-focused control interfaces, intuitive demo-booth robots, VR/AR-adjacent control schemes, teaching differential-drive kinematics.

## Hardware & Pin Mapping
Only the built-in motor driver, RGB, and buzzer are used. Motor A (IN1=11, IN2=10, PWMA=12) = **left wheel**, Motor B (IN3=46, IN4=13, PWMB=3) = **right wheel** — swap these two roles in code if your wiring is mirrored.

## Dashboard
- Tilt "bubble" visualizer (moves within a circular track as you tilt)
- **Enable Motion Control** button (required on iOS 13+ to grant motion-sensor permission; auto-starts on Android)
- **Recalibrate Center** button — zero out your current phone angle as "neutral" so you don't have to hold it perfectly flat
- Sensitivity slider (15°–45°) — sets how much tilt equals full speed
- Live left/right motor speed readout

## Proportional Drive Logic (tank-style mixing)
```
throttle = map(betaTilt,  -maxTilt..maxTilt, -255..255)   // front/back
steer    = map(gammaTilt, -maxTilt..maxTilt, -255..255)   // left/right
left  = constrain(throttle + steer, -255, 255)
right = constrain(throttle - steer, -255, 255)
```
A ±5° deadzone on both axes prevents idle jitter from creeping the rover forward.

## RGB / Buzzer
- Green = idle (both wheel speeds 0), Blue = moving, Purple = boot
- Short click the instant motion starts from a stop

## Automation / Safety Rules
- **Dead-man safety**: if no tilt update arrives for 400ms (phone locked, browser tab backgrounded, WiFi drop) motors stop immediately — tighter than the D-pad projects because tilt is a continuous stream, so a gap means something went wrong
- Client-side throttling sends tilt updates at 10Hz to avoid flooding the WebSocket

## Data Logging & OTA
Same LittleFS `/log.csv` pattern (connect/disconnect + safety-stop events); OTA via `ArduinoOTA` hostname `linkx-gesturebot`.

## Testing Procedure
1. Flash + upload `/data`, connect phone to `LinkX-GestureBot` WiFi, open `192.168.4.1`
2. Tap **Enable Motion Control** (grant permission if prompted on iOS)
3. Hold phone naturally, tap **Recalibrate Center**
4. Tilt forward — confirm both wheels turn forward proportional to tilt angle, RGB blue
5. Tilt phone to one side while tilting forward — confirm one wheel speeds up, the other slows/reverses (turning behavior)
6. Return phone to level — confirm rover stops, RGB green
7. Lock the phone screen mid-drive — confirm auto-stop within ~400ms

## Alternative: Wearable MPU6050
To use a physical MPU6050 instead of the phone's sensors:
1. Wire MPU6050 to SDA=GPIO8, SCL=GPIO9 (3.3V, GND)
2. Add `Wire.begin(8,9)` + an MPU6050 library (e.g. `Adafruit_MPU6050`) in `setup()`
3. In `loop()`, read accelerometer pitch/roll instead of receiving `{cmd:'tilt',...}` over WebSocket, and call `applyTilt()` directly with those values
4. Remove the phone-side `deviceorientation` listener from `index.html` (dashboard becomes a read-only status display instead)

## Folder Structure
```
gesture_robot/
├── gesture_robot.ino
├── data/
│   └── index.html
└── README.md
```

## Libraries
`ESPAsyncWebServer`, `AsyncTCP`, `ArduinoJson`, `LittleFS`, `ArduinoOTA` (bundled). No extra library needed for the phone-sensor version; add `Adafruit_MPU6050` + `Adafruit_Sensor` only if you switch to the wearable alternative.

## Future Improvements
- Add a "speed cap" slider for beginners/kids
- Smooth motor speed changes with a ramp filter to avoid jerky direction reversals
- Add haptic feedback (`navigator.vibrate()`) on the phone when an auto-stop occurs
