## R2-D2 Brewbot

Simple Arduino sketches to control a pour spout using an ultrasonic sensor, a servo, and a relay. The default sketch is focused on the spout only and is easy to configure for non-coders.

### Sketches
- **SpoutOnly.ino**: Pour-only control using an ultrasonic sensor. Uses a `shouldPour()` function for clean, readable logic with edge-triggering and a maximum pour time.
- **HeadAndSpout.ino**: Combined example (head movement + spout). Optional.
- **HeadSpin.ino**: Head-only example. Optional.

### How it works (SpoutOnly.ino)
1. Reads distance from an ultrasonic sensor (HC-SR04/PING style) and averages a few readings for stability.
2. When a cup moves into range, pouring starts (edge-triggered: far → near).
3. Pouring stops when either:
   - The cup moves away, or
   - The maximum pour time elapses.
4. Pour won’t restart until the cup leaves and returns (re-armed).

### Wiring (default pins)
- Ultrasonic TRIGGER → Arduino `2`
- Ultrasonic ECHO → Arduino `4`
- Servo signal → Arduino `9` (Power from a stable 5V source; do NOT power a large servo from the Arduino 5V pin)
- Relay signal → Arduino `13` (Many relay modules are active-LOW)

Power notes:
- Share ground between Arduino, servo power supply, and relay module.
- If your relay is active-HIGH, invert the HIGH/LOW in `pourOn()`/`pourOff()`.

### Configuration (top of SpoutOnly.ino)
- Pins: `TRIGGER_PIN`, `ECHO_PIN`, `SERVO_PIN`, `RELAY_PIN`
- Servo angles: `POUR_OPEN_ANGLE`, `POUR_CLOSED_ANGLE`
- Trigger distance: `POUR_DISTANCE_CM` (cup considered “present” when closer than this)
- Max pour time: `MAX_POUR_TIME_MS` (automatic stop)
- Sampling: `SAMPLE_COUNT`, `SAMPLE_DELAY_MS` (stability vs. responsiveness)
- Debug: `ENABLE_SERIAL_DEBUG` (true/false) and `SERIAL_BAUD_RATE` (115200)

### Build & Upload
1. Open `SpoutOnly.ino` in the Arduino IDE.
2. Select your board and port.
3. Click Upload.
4. (Optional) If `ENABLE_SERIAL_DEBUG` is true, open Serial Monitor at `115200` baud to see distance and pour events.

### Calibration Tips
- Adjust `POUR_OPEN_ANGLE` / `POUR_CLOSED_ANGLE` for your specific spout hardware.
- Tune `POUR_DISTANCE_CM` so the cup triggers reliably but false triggers are rare.
- Increase `SAMPLE_COUNT` or `SAMPLE_DELAY_MS` if readings are noisy.
- If the relay logic seems inverted (on when it should be off), swap the `HIGH`/`LOW` in `pourOn()`/`pourOff()`.

### Safety
- Liquids and electronics don’t mix—keep wiring and boards protected from spills.
- Use a proper external 5V supply for the servo and share ground with the Arduino.


