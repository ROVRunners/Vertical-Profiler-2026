# Vertical Profiler (VP)

This project is a small autonomous vertical profiler built around an ESP32-C3, a buoyancy engine actuator, a Bar02 pressure sensor, an RTC, and a NeoPixel status display. The system creates its own Wi-Fi access point, hosts a local control page, runs a mission profile in the water, logs depth over time, and displays live status information to the operator.

The VP is designed to move between target depths, hold within a specified tolerance band, and record its motion for later review. It also includes manual mode, onboard visual status indicators, and live PID tuning through the client page.

---

## System Overview

The VP is a depth-controlled profiling platform. It uses a pressure sensor to estimate depth and a buoyancy engine actuator to drive the vehicle upward or downward in the water column. A control loop compares the current depth to a target depth and adjusts the buoyancy engine command to reduce the error.

The system can operate in three main ways:

- **Idle mode**: waiting for user input
- **Automatic mission mode**: running the programmed depth profile
- **Manual mode**: operator directly commands actuator microseconds

The VP serves a simple web interface over its own Wi-Fi access point. From that page, the operator can:

- start a mission
- stop logging
- view live depth and pressure
- view elapsed mission time
- tune PID values live
- enter manual mode
- directly command actuator microsecond values
- retrieve and graph logged depth data

---

## Hardware Used

This project is currently built around the following hardware:

- **SparkFun Pro Micro ESP32-C3**
- **Blue Robotics Bar02 pressure sensor**
- **RTC module**
- **Servo-style buoyancy engine actuator**
- **7-pixel NeoPixel Jewel**
- **External power system for actuator and electronics**

---

## Main Features

- Local Wi-Fi access point and web control page
- Live depth and pressure readout
- Automatic multi-step vertical profiling mission
- Hold logic based on remaining within depth tolerance for the full hold period
- Depth logging and graphing in the browser
- Live PID tuning from the client
- Manual actuator control mode
- NeoPixel visual feedback for motion, state, depth error, and actuator command
- Depth zeroing before mission start

---

## How the VP Works

### 1. Startup

When the VP powers on, it initializes the sensors, LED ring, and actuator. During startup it also performs a buoyancy engine prime cycle. This helps clear and prepare the buoyancy engine before the vehicle begins normal operation.

After startup, the board creates a Wi-Fi access point. The operator connects to this network and opens the control page in a browser.

### 2. Depth Sensing

Depth is measured using the Bar02 pressure sensor. The sensor reports pressure, and the software converts that pressure into depth using the configured fluid density.

Before a mission starts, the VP performs a **depth zeroing step**. It samples the pressure sensor several times, averages those readings, and stores that average as the depth zero offset. All later depth readings are referenced to that zero point.

That means:

- `0.00 m` represents the depth at the moment the vehicle was zeroed
- positive depth means deeper than the zeroing point

### 3. Mission Control

When the operator presses **Start Logging**, the VP:

1. exits manual mode if needed
2. zeroes the depth sensor
3. enters the first descent state
4. begins logging depth over time

The mission state machine then moves through the programmed sequence of target depths and hold periods.

### 4. PID Depth Control

The VP uses a PID loop to command the buoyancy engine. The controller compares:

- **target depth**
- **measured depth**

and computes an output in actuator microseconds.

The PID terms mean:

- **P (proportional)**: reacts to present depth error
- **I (integral)**: reacts to accumulated error over time
- **D (derivative)**: reacts to how quickly the error is changing

This output is then translated into a microsecond command sent to the buoyancy actuator.

### 5. Logging

The VP logs:

- elapsed time since mission start
- depth

These are stored in RAM during the mission and sent to the web client when requested. The web page plots **depth vs time** so the operator can see the vehicle profile.

---

## Mission Logic

The mission sequence is organized as a state machine.

A typical mission may look like this:

1. descend to deep target
2. hold for 30 seconds
3. ascend to shallow target
4. hold for 30 seconds
5. repeat as required
6. either hold shallow or self-recover to surface

### Important Hold Behavior

The VP does **not** simply wait 30 seconds after reaching a target once. It must stay **within the allowed target tolerance band continuously** for the full hold time.

If it drifts outside the tolerance:

- the hold timer resets
- the VP must re-enter and remain inside the band again for the full hold period

This behavior is important because it ensures the vehicle truly holds station at the required depth rather than just briefly touching the target.

---

## Manual Mode

Manual mode is intended for direct operator testing and troubleshooting.

When manual mode is enabled:

- automatic mission logic is suspended
- the operator can directly write actuator microsecond values
- the actuator holds the commanded value until changed or manual mode is exited

This is useful for:

- testing the buoyancy engine
- checking actuator direction
- finding neutral or trim points
- verifying wiring and hardware behavior

---

## Web Interface

The VP hosts a browser-based control page. This is the main operator interface.

### Main Controls

- **START LOGGING**  
  Begins the mission, zeroes depth, and starts depth logging.

- **STOP LOGGING**  
  Stops logging and returns the VP to idle behavior.

### Status Display

The page shows:

- **Elapsed time**
- **RTC time**
- **Pressure**
- **Depth**
- **Current state**
- **Sample count**
- **Actuator microseconds**

### PID Tuning Panel

The PID panel allows the operator to edit:

- `Kp`
- `Ki`
- `Kd`

without reflashing the firmware.

When applied, the new values take effect immediately in RAM.

### Manual Control Panel

The manual control panel allows the operator to:

- enable manual mode
- disable manual mode
- enter a raw actuator microsecond command

---

## What the Information Means

### Elapsed Time

This is the mission runtime in seconds. It shows how long the VP has been logging since the mission started.

### RTC Time

This is the current date and time from the onboard RTC. It is useful for correlating mission runs and system events.

### Pressure

This is the raw pressure sensor reading in mbar. It is the measured fluid pressure and is the source value used to derive depth.

### Depth

This is the zero-referenced depth in meters.

Examples:

- `0.00 m` = same depth as the point where the mission was zeroed
- `0.40 m` = 40 cm below the zero point
- `0.60 m` = 60 cm below the zero point

### State

The state tells you what the VP is trying to do right now.

Examples:

- `IDLE` = waiting
- `DESCEND_*` = driving deeper
- `ASCEND_*` = driving shallower
- `HOLD_*` = maintaining a target depth
- `STATION_KEEP_*` = remaining at target after the profile
- `RECOVER_SURFACE` = recovering upward
- `MANUAL` = operator-controlled actuator mode

### Samples

This is the number of logged depth samples currently stored in memory.

### Actuator Command

This is the actual microsecond command being sent to the buoyancy engine actuator.

This value is important because it tells you what the controller is trying to do physically.

---

## NeoPixel LED Meanings

The VP uses a 7-pixel NeoPixel Jewel as a compact visual status display. Each section of the ring has a different purpose.

### LEDs 1–2: Vertical Velocity

These LEDs show how the VP is moving in the water.

- **Yellow** = moving upward
- **Purple** = moving downward
- stronger color = larger vertical speed

The display is scaled to about **0.2 m/s** maximum expected velocity.

This means:

- a faint color indicates slow movement
- a strong color indicates faster movement
- yellow tells you the profiler is rising
- purple tells you the profiler is descending

### LED 3: State Indicator

This LED shows the current operating state of the VP.

- **Blinking blue** = idle
- **Magenta** = descending
- **Red** = deep hold
- **Amber** = ascending
- **Green** = shallow hold
- **White** = station keeping

This LED is meant to give quick, high-level mission awareness.

### LEDs 4–5: Target Depth Position

These LEDs show how close the VP is to the current target depth band.

- **Bright pink** = exactly on target
- **Pink fading to black** = within tolerance but farther from target
- **Blinking red** = outside the allowed target tolerance

These LEDs are especially useful during hold states because they show whether the VP is actually centered on the target or just barely staying within bounds.

### LEDs 6–7: Buoyancy Engine Command

These LEDs show the actuator command being sent to the buoyancy engine.

- **Blue** = low end of the command range
- **Orange** = high end of the command range
- the color shifts gradually between the two

This gives immediate visual feedback about what the controller is asking the actuator to do.

### Manual Mode Override

In manual mode, **all 7 LEDs blink yellow**.

This overrides the normal LED meanings and clearly indicates that automatic mission control is not active.

### Depth Zeroing Indication

During the depth zeroing process before a mission begins, **all 7 LEDs blink yellow quickly**.

This shows that the vehicle is in a short startup calibration phase before it begins diving.

---

## Target Tolerance

The VP uses a target tolerance band around each depth setpoint.

For example, if the target is:

- `0.40 m`
- tolerance = `±0.05 m`

then the acceptable band is:

- `0.35 m` to `0.45 m`

The profiler must remain inside that band continuously for the hold timer to complete.

This matters because simply crossing the target is not enough. The VP must demonstrate stable control at the desired depth.

---

## Logging Parameters

The VP stores depth logs in memory. The two main parameters are:

- `MAX_SAMPLES`
- `SAMPLE_INTERVAL_MS`

These determine:

- how often data is logged
- how long a full mission can be recorded before memory fills

For example:

- smaller sample interval = more detail, shorter total runtime
- larger sample interval = less detail, longer total runtime

A good balanced setting is often around **250 ms** sample interval if you want decent profile detail without using too much memory.

---

## Typical Operating Procedure

1. Power on the VP.
2. Wait for startup and buoyancy engine priming to finish.
3. Connect to the VP Wi-Fi access point.
4. Open the client page in a browser.
5. Verify live pressure, depth, and system status.
6. Adjust PID values if needed.
7. Press **START LOGGING**.
8. Wait while the VP zeroes depth.
9. Let the VP run its programmed profile.
10. Retrieve the VP when the mission is complete.
11. Press **STOP LOGGING**.
12. Review the depth graph and saved data.

---

## Tuning Notes

### PID Tuning

If the VP oscillates too much:

- reduce `Kp`
- add or increase `Kd`

If the VP responds too weakly:

- increase `Kp`

If it consistently sits above or below the target:

- increase `Ki` slightly

### Manual Mode

Manual mode is often the fastest way to:

- confirm actuator direction
- verify your microsecond range
- check if the buoyancy engine is behaving as expected

### Depth Zeroing

Always zero the VP at the correct reference depth before starting a mission. If the zero point is wrong, every depth reading during the mission will be shifted.

---

## Safety and Operational Notes

- Do not assume manual mode is safe for autonomous use.
- Make sure the actuator command range is correct for your hardware.
- Verify control direction before running a full mission.
- Confirm that the pressure sensor is stable before starting.
- Watch the LED indications during testing, especially target-bound and manual-mode indicators.

---

## Future Improvements

Potential future additions include:

- persistent storage of PID values
- download/export of logged mission data
- additional logged channels such as pressure, velocity, and actuator command
- improved graphing and mission summary tools
- mission configuration from the web client

---

## Repository Purpose

This repository documents the firmware, operator interface, and control behavior of the Vertical Profiler. Its purpose is to make the VP easy to understand, operate, test, and improve for future missions.
