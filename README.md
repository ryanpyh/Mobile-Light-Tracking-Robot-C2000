# Mobile Light-Tracking Robot (TI C2000)

Light-tracking differential-drive robot developed for **UIUC ME 461**. The robot uses a TI C2000 LaunchPad for real-time sensing and control, a four-direction photodiode array to estimate the direction of a light source, wheel encoders for feedback, and a cascaded controller to steer and drive toward the source.

## Features

- **Four-direction light sensing** using north, south, east, and west photodiodes connected to the C2000 ADC.
- **Real-time light tracking** with:
  - normalized east-west directional error,
  - proportional-derivative steering control,
  - derivative low-pass filtering,
  - deadband and output saturation,
  - hysteresis for detecting and losing the light source.
- **Differential-drive motor control** with:
  - inner PI wheel-speed loops,
  - coupled left/right turning control,
  - PWM motor commands,
  - anti-windup and stopped-state integrator reset.
- **Search and recovery behavior** when the light source is lost.
- **Rear-light handling** that commits the robot to a controlled spin when the south-facing sensor indicates that the source is behind the robot.
- **Wheel odometry** from eQEP1/eQEP2 quadrature encoder inputs.
- **IMU support** through SPI, including accelerometer and gyroscope calibration and a Kalman filter for tilt estimation.
- **CAN communication** for distance, light-level, and measurement-quality data from external sensors.
- **Serial communication** for debugging and LabVIEW data exchange.
- **LED and buzzer feedback** for calibration, status, and close-range light detection.

## Control architecture

The control loop is organized as a cascaded system:

1. **Sensor acquisition**
   - ADC conversions sample the four photodiode channels.
   - A 21st-order FIR low-pass filter can be applied to the sensor signals.
   - Startup calibration estimates offsets for the light sensors and IMU.

2. **Light-direction estimation**
   - The east-west error is calculated as `E - W`.
   - A directionality magnitude is calculated from the north/south and east/west differences.
   - The east-west error is normalized by the directionality magnitude so the robot can still estimate direction as the source becomes weaker.

3. **Outer steering loop**
   - A PD controller converts the normalized light-direction error into a turn command.
   - The derivative term is smoothed with a first-order IIR filter.
   - Deadband, diagonal-sensor compensation, committed spin logic, and maximum turn-rate limiting reduce oscillation and unwanted motor reversals.

4. **Forward-speed command**
   - The robot moves forward only when the source is detected in front of it and the east-west error is sufficiently small.
   - Forward speed is reduced during aggressive turns.
   - The robot stops when the north-facing sensor indicates that the light is very close.

5. **Inner wheel-speed loops**
   - Encoder-derived wheel velocities are compared with the forward and turning commands.
   - PI control produces left and right motor efforts.
   - Motor commands are saturated before being mapped to ePWM duty cycles.

The main implementation is in [`final_project_alt.c`](final_project_alt.c), with the principal control logic in `SWI_isr()` and sensor/motor support distributed across the interrupt service routines and initialization functions.

## Hardware and peripherals

The firmware is configured for a TI C2000 F28379D-class LaunchPad and uses the following interfaces:

| Subsystem | Interface / peripheral | Purpose |
| --- | --- | --- |
| Light array | ADCA channels A2-A5 | West, east, south, and north photodiode measurements |
| Motors | ePWM2A/ePWM2B | Left and right motor PWM commands |
| Wheel encoders | eQEP1/eQEP2 | Wheel position and velocity feedback |
| IMU | SPIB | Accelerometer and gyroscope measurements |
| External ranging/light sensors | CANB | Distance, light level, and signal quality data |
| Debugging | SCIA | Human-readable serial diagnostics |
| LabVIEW link | SCID | Binary telemetry and command exchange |
| Status output | GPIO LEDs and ePWM9A | Calibration, activity, and buzzer feedback |

Pin assignments and peripheral setup are defined in `final_project_alt.c`. Verify the wiring and board revision before applying this firmware to hardware.

## Repository layout

- `final_project_alt.c` — application entry point, peripheral initialization, interrupt handlers, sensor processing, state machine, and control loops.
- `F28379dCAN.c`, `F28379dCAN.h` — CAN initialization and message handling support.
- `F28379dSerial.c`, `F28379dSerial.h` — serial port and LabVIEW communication support.
- `F2837xD_Adc.c` — C2000 ADC support code.
- `F2837xD_EPwm.c` — ePWM support code.
- `F2837xD_Spi.c` — SPI support code.
- `F2837xD_CpuTimers.c` — CPU timer support code.
- `LEDPatterns.c`, `LEDPatterns.h` — LED display helpers.
- `buffer.h` — circular buffers used by the serial interfaces.
- `device/` — device headers and driver-library support files.
- `targetConfigs/` — Code Composer Studio target configuration files.
- `*.cmd` — linker command files for RAM and flash builds.
- `.project`, `.cproject`, `.ccsproject` — Code Composer Studio project metadata.

## Building and loading

This project is structured as a **Code Composer Studio (CCS)** C2000 project.

1. Install Code Composer Studio with support for TI C2000 devices.
2. Clone this repository and import it as an existing CCS project.
3. Connect the TI C2000 LaunchPad and select the appropriate target configuration from `targetConfigs/`.
4. Select the desired linker configuration:
   - `2837xD_RAM_lnk_cpu1.cmd` for development/debugging from RAM.
   - `2837xD_FLASH_lnk_cpu1.cmd` for a flash build.
5. Build the project in CCS.
6. Load the generated program onto CPU1 and run it with the robot lifted or otherwise safely restrained during initial testing.

The firmware assumes a **200 MHz LaunchPad CPU clock**. Do not change the `LAUNCHPAD_CPU_FREQUENCY` setting without also reviewing timer, PWM, ADC-trigger, SPI, and serial timing calculations.

## Startup and calibration

At startup, the firmware initializes the C2000 peripherals, ADC, PWM outputs, serial ports, SPI, CAN, encoder interfaces, and interrupts. The IMU and light-sensor calibration sequence then proceeds through startup states before motor commands are enabled.

During calibration:

- Keep the robot stationary.
- Keep the light-sensor environment stable.
- Do not place the robot against an obstacle or allow the wheels to drive freely.
- Wait for calibration to complete before evaluating tracking behavior.

The blue LaunchPad LED is used as a calibration indicator, and serial diagnostics can be used to monitor sensor values, tracking state, turn strength, and other runtime variables.

## Runtime behavior

The robot generally follows this state sequence:

1. **Searching** — rotates in place to locate a sufficiently strong light source.
2. **Tracking** — uses the outer light PD controller and inner wheel PI controllers to align with and approach the source.
3. **Committed spin** — performs a short, fixed-direction spin when the source is strongly off-axis or detected behind the robot.
4. **Target reached** — stops forward motion and activates the buzzer when the front sensor indicates that the source is very close.

The implementation includes hysteresis and debounce counters so brief sensor fluctuations do not immediately switch between searching and tracking.

## Tuning parameters

The primary control and behavior parameters are declared near the top of `final_project_alt.c`. Important parameters include:

- `Kp_light`, `Ki_light`, `Kd_light` — outer light-tracking controller gains.
- `deadband` — normalized steering-error region that is ignored.
- `max_turn_rate` — maximum commanded turn rate.
- `m_tracklimit_start`, `m_tracklimit_stop` — tracking-state hysteresis thresholds.
- `front_sensor_follow_limit` — front-sensor limit for allowing forward travel.
- `front_sensor_prox_limit` — close-target limit that stops the robot and starts the buzzer.
- `Kp`, `Ki` — inner wheel PI gains.
- `Kp_turn` — coupling gain for differential turning.
- `Vref` and the forward-speed assignments — desired robot speed.

Tune one loop at a time. Confirm sensor polarity and encoder direction first, then tune the wheel-speed loops, and finally tune the outer light-tracking loop at low speed.

## Debugging and telemetry

SCIA prints human-readable diagnostics such as:

- north, east, west, and south sensor values,
- forward-motion state,
- spin command,
- turn strength.

SCID is used for LabVIEW communication. The firmware periodically packages telemetry including robot position estimates, light directionality, sensor values, and normalized steering error. LabVIEW can also provide runtime forward-speed and turn commands through the serial interface.

## Safety and limitations

- Test with the wheels off the ground before making changes to control gains or polarity.
- Confirm that positive and negative PWM commands produce the expected wheel directions.
- Confirm encoder signs before enabling closed-loop control.
- The code contains hardware-specific pin mappings, timing assumptions, sensor polarity assumptions, and calibration constants.
- The repository does not currently include a full wiring diagram, bill of materials, automated tests, or a reproducible non-CCS build system.
- Validate all sensor thresholds and controller gains on the actual robot; values that work in one lighting or floor condition may not generalize.

## Project context

This repository contains the embedded C firmware for a UIUC ME 461 team project. It combines laboratory exercises and project-specific code for sensing, motor control, state estimation, communication, and autonomous light tracking on a TI C2000 LaunchPad.
