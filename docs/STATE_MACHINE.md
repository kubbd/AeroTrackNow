# STATE MACHINE

## States
0. **BOOT**
   - Init sensors, radio, GNSS, SD
   - Servos neutral
   - Wait for TARGET packet

1. **WAIT_FOR_DROP**
   - Servos neutral
   - Wait for drop event
   - Drop detection is gated by descent (**sink_rate > 1.0 m/s**) to avoid rocket-launch false triggers.

2. **WAIT_FOR_STABLE_DESCENT**
   - Servos neutral
   - Wait until sink rate is stable (within band) continuously for ~2.5 s

3. **GUIDED_DESCENT**
   - Predictive guidance active
   - Compute predicted landing point and steer to reduce error

4. **TERMINAL**
   - Servos frozen neutral below terminal height and within capture radius
   - No more steering

5. **LANDED**
   - Servos neutral

## Transitions
- BOOT → WAIT_FOR_DROP: TARGET received
- WAIT_FOR_DROP → WAIT_FOR_STABLE_DESCENT: dropDetected() == true
- WAIT_FOR_STABLE_DESCENT → GUIDED_DESCENT: descentStable() == true
- GUIDED_DESCENT → TERMINAL: height < TERMINAL_HEIGHT_M && predicted error < CAPTURE_RADIUS_M
- TERMINAL → LANDED: landingDetected() == true
