# CONTEXT.md — Pick-and-Place Robot Arm (G06)

Domain glossary for the single-axis pick-and-place robot arm firmware. Implementation details belong in code comments, not here.

---

## Robot Arm

The physical single-axis robotic arm driven by a DC gearmotor. One rotational degree of freedom. Carries a Gripper at its end-effector.

---

## Motor

The DC gearmotor that moves the Robot Arm. Has a 70:1 gear ratio. Receives voltage commands (expressed as PWM duty cycle) from the firmware.

---

## Encoder

Quadrature sensor (2048 pulses/rev) mounted on the Motor output shaft. The authoritative source of position and velocity for the control loop.

---

## Home Position

The absolute reference point (0°) for all position commands. Established once per power-on by the Homing Sequence. All positions are expressed as degrees from Home.

---

## Homing Sequence

The startup procedure that drives the arm until the Proximity Sensor detects the reference mark, then declares that point as Home Position. Must complete before position-based commands are valid.

---

## Proximity Sensor

The inductive or optical sensor that detects the arm's reference mark during the Homing Sequence.

---

## Gripper

The end-effector attached to the Robot Arm. Composed of two independent spring-return actuators:

- **Vertical actuator** — moves the gripper UP (energised) or DOWN (spring return).
- **Claw actuator** — CLOSES the claw (energised) or OPENS it (spring return).

All four combinations (UP+OPEN, UP+CLOSE, DOWN+OPEN, DOWN+CLOSE) are independently reachable.

---

## Pick Sequence

An automated Gripper sequence that closes the claw and lifts to pick an object. Triggered as a unit; individual actuator steps are not user-visible during execution.

---

## Place Sequence

An automated Gripper sequence that lowers, opens the claw, and retracts to place an object. Triggered as a unit.

---

## Reed Switch

A magnetic feedback sensor that reports whether a Gripper actuator has physically reached its target position (UP, DOWN, OPEN, or CLOSED).

---

## E-Stop (Emergency Stop)

A latching safety state that immediately halts the Motor. Once latched, motion is impossible until the operator explicitly clears it. Multiple sources can trigger an E-Stop (see Fault).

---

## Fault

A bitmask flag that identifies the cause of an E-Stop or a safety event. A Fault is not the same as an E-Stop — a Fault describes *why* the E-Stop happened. Multiple Faults can be active simultaneously.

| Fault | Meaning |
|---|---|
| MOTOR_STALLED | Motor has power applied but no movement for ≥ 2 s |
| ENCODER_ERROR | Encoder signal lost or inverted under load |
| JOYSTICK_LOST | Joystick link silent for ≥ 200 ms or 5 consecutive bad packets |
| OVER_ROTATION | Arm exceeded the Soft Limit |
| ESTOP_PHYSICAL | Physical E-Stop button pressed |
| PROX_LOST | Proximity Sensor open |
| ESTOP_JOYSTICK | Joystick safety button pressed |
| ESTOP_DASHBOARD | Dashboard emergency stop button |
| ESTOP_MODBUS | Base System soft-stop command |

---

## Soft Limit

The maximum rotation allowed from Home Position before FAULT_OVER_ROTATION is raised. Default: 720° (2 full turns). Protects cables from over-winding.

---

## Stall

The condition where the Motor has significant PWM applied but velocity remains near zero for longer than the stall timeout. Indicates a mechanical blockage or overload.

---

## Control Mode

The operating mode of the Motor controller at any instant. Modes: STOPPED, SPEED, POSITION, AUTOTUNE, AUTOTUNE_SPEED, TEST, GHOST, HOMING.

---

## System Mode

Which external control source is currently active. Exactly one source is active at a time:

- **JOYSTICK** — Joystick and Dashboard control the arm. LPUART1 carries Dashboard telemetry at 115200 8N1.
- **BASE_SYSTEM** — Base System controls the arm via Modbus RTU. LPUART1 switches to 19200 8E1.

---

## Jog Mode

The sensitivity setting for Joystick step/jog commands. Either COARSE (large steps, fast jog) or FINE (small steps, slow jog).

---

## Trajectory

The planned motion profile generated for each move: a time-series of position, velocity, and acceleration setpoints following an S-curve shape. The control loop tracks this profile, not the raw target position directly.

---

## S-Curve

The trajectory shape that limits jerk (rate of change of acceleration) to produce smooth, vibration-reduced motion. Defined by three parameters: maximum velocity, maximum acceleration, and maximum jerk.

---

## Cascade PID

The two-loop control structure: an outer Position PID loop produces a velocity setpoint, which an inner Speed PID loop tracks. Feedforward terms augment the speed loop.

---

## Feedforward

Voltage terms added to the Speed PID output based on Trajectory velocity (K_vff), acceleration (K_aff), and Kalman-estimated load torque (K_tff). Reduces reliance on integrator windup.

---

## Kalman Filter

A 4-state observer running at 1 kHz that estimates position, velocity, load torque, and armature current from Encoder measurements and the PWM voltage input. Estimates are used for Feedforward and diagnostics.

---

## ZVD Input Shaper

A Zero Vibration with Derivative filter applied to position commands to suppress residual arm vibration at the arm's natural frequency.

---

## Dashboard

The static HTML/JS application (Chrome/Edge only, Web Serial API) that connects to the STM32 over LPUART1 when in JOYSTICK System Mode. Used for telemetry display, PID tuning, gripper commands, and test modes.

---

## Telemetry

The `$KEY:VAL,...*\r\n` packet protocol transmitted by the STM32 to the Dashboard at 50 Hz (fast) and 1 Hz (slow config). Also used for incoming commands from the Dashboard.

---

## Joystick

The ESP32-based gamepad that communicates with the STM32 over USART3. Always active regardless of System Mode. Sends single-character commands and status packets.

---

## Base System

The external automation system that commands the robot arm via Modbus RTU on LPUART1 when in BASE_SYSTEM System Mode.

---

## Ghost Mode

A test mode that records a motion trajectory into a buffer and replays it on command. Used to characterise and compare control behaviour across tuning changes.

---

## Sine Test

A test mode that commands sinusoidal velocity inputs to the Motor for frequency-response characterisation.

---

## Autotune

An automated relay-feedback procedure that estimates PID gains for either the Position loop or the Speed loop.

---

## Current Sensor

The WCS1800 Hall-effect sensor on the Motor supply line. Reports Motor current in Amperes. Used for overcurrent E-Stop and Kalman Filter input.
