# Cytron MDD10A Motor Driver (Replaces the L298N)

Firmware: `velocity_controller/` (MDD10A). The old L298N version is kept in `velocity_controller_l298n/`.

## Wiring

| From | To (MDD10A) | Notes |
|---|---|---|
| Battery + | POWER + | Through the robot's power switch, plus an inline fuse (10–15 A) if you have one. Thick wire. |
| Battery − | POWER − | Thick wire. |
| Left motors (old L298N OUT1) | M1A | Keep the same order as on the L298N. |
| Left motors (old L298N OUT2) | M1B | |
| Right motors (old L298N OUT3) | M2A | |
| Right motors (old L298N OUT4) | M2B | |
| Arduino D5 | PWM1 | Left speed (same pin as the old LEFT_PWM / ENA). |
| Arduino D12 | DIR1 | Left direction (was LEFT_IN1). |
| Arduino D6 | PWM2 | Right speed (same pin as the old RIGHT_PWM / ENB). |
| Arduino D7 | DIR2 | Right direction (was RIGHT_IN1). |
| Arduino GND | GND (control header) | **Required**: common ground for the PWM/DIR signals. |

- **Freed pins:** D13 and D8 (old LEFT_IN2 / RIGHT_IN2) are no longer used; leave them unconnected.
- **Unchanged:** encoders (D2, D4, D3, D9) and the Arduino's USB to the Pi.
- **Power:** the Arduino is powered over USB from the Pi. Do not connect anything from the MDD10A to the Arduino's 5V/VIN pins. If the Arduino was previously fed from the L298N's 5V output, remove that wire.
- **Battery voltage:** the MDD10A takes 5–30 V. Your motors now get almost the full battery voltage (the L298N lost about 2 V), so they will run a bit faster at the same PWM.

## Bring-up (wheels off the ground)

1. **Battery disconnected.** Flash the new firmware with the Uno on the Pi's USB, robot launch stopped:

   ```bash
   arduino-cli compile --upload -p /dev/ttyACM0 --fqbn arduino:avr:uno ~/ros2_ws/src/Autonomous_Vehicle/firmware/velocity_controller
   ```

2. Do the wiring above with the battery still disconnected.
3. **Test the driver alone.** Prop the robot up so no wheel touches anything. Connect the battery and press the MDD10A's on-board test buttons (M1A/M1B, M2A/M2B). Each side's wheels should spin in both directions. If not, check the power and motor wires before involving the Arduino.
4. **Test forward.** Start `bringup.launch.py` (or navigation), run teleop and tap `i` (forward):
   - Both sides spin forward: done.
   - One side spins backwards: flip `LEFT_FORWARD_DIR` or `RIGHT_FORWARD_DIR` (LOW to HIGH) at the top of the sketch and re-flash. Alternatively swap that side's two motor wires on the MDD10A. Do not change the encoder wiring.
5. **Check the encoders still agree.** Driving forward, both encoder counts must go **up**. Watch with:

   ```bash
   ros2 topic echo /encoder_counts
   ```

6. **Tune `MIN_PWM`** (default 70): it should be the lowest value at which a slow turn starts reliably. Raise it in steps of 5 if slow turns stall, or lower it if the robot jerks at low speed.
7. Put it on the floor and test turns and a Nav2 goal. After a few minutes of driving, the MDD10A should only be slightly warm.
