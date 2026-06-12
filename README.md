# Vertical Profiler 2026

This repository contains the design files, firmware, and project documentation for our 2026 Vertical Profiler.

The profiler was built for the 2026 MATE ROV competition. The goal was to make a small, reliable vertical profiler that could work in a cold glycol environment, survive the required depth, and still stay simple and affordable.

The final design is a transparent PVC float with a syringe-based buoyancy engine, ESP32-C3 control board, Bar02 pressure sensor, RTC, onboard NeoPixel status display, and a Wi-Fi interface for mission control and tuning.

## Main features

- Rated for 5 m depth
- Designed for low-temperature operation
- Transparent PVC enclosure
- Syringe-based buoyancy engine
- ESP32-C3 based control system
- Blue Robotics Bar02 pressure sensor
- RTC for onboard timekeeping
- NeoPixel LED status display
- Wi-Fi access point control page
- Live telemetry and depth graph
- Adjustable mission settings from the client
- Adjustable PID settings from the client
- Manual actuator control mode
- Logged depth-over-time mission data

The current firmware runs the profiler as its own Wi-Fi access point, uses I2C for the RTC and pressure sensor, drives the buoyancy actuator with servo-style PWM, and supports live mission control, PID tuning, manual mode, data logging, and LED state feedback. :contentReference[oaicite:0]{index=0}

## How it works

The profiler changes depth by moving a buoyancy engine. A pressure sensor is used to estimate depth, and the control loop adjusts the actuator command to move toward the target depth.

Before a mission starts, the depth sensor is zeroed. Once started, the profiler runs a programmed sequence of depth targets and hold times. The current firmware is set up for a deep target, a shallow target, and an optional self-recovery to the surface after the final hold. :contentReference[oaicite:1]{index=1}

The current mission profile is:

1. Descend to the deep target
2. Hold depth for the required hold time
3. Ascend to the shallow target
4. Hold depth again
5. Repeat the cycle
6. Either stay shallow or recover to the surface, depending on mission settings

The default firmware values currently use a deep target of 2.50 m, a shallow target of 0.40 m, a surface target of 0.02 m, a 32 second hold time, and timeout limits for transit, hold, and recovery states. :contentReference[oaicite:2]{index=2}

## Control interface

The profiler hosts a browser-based control page over Wi-Fi, so there is no separate base station required. The web interface includes:

- Start and stop mission controls
- Live elapsed time
- RTC readout
- Live pressure and depth
- State and actuator readout
- Depth-over-time graph
- Mission setting controls
- PID tuning controls
- Manual actuator mode

The current firmware uses the SSID `VP_Float` and serves the control interface from the ESP32-C3 in AP mode. :contentReference[oaicite:3]{index=3}

## LED feedback

The 7-pixel NeoPixel Jewel is used as a quick operator display.

- LEDs 1–2 show vertical velocity
- LED 3 shows system state
- LEDs 4–5 show how close the profiler is to the current target depth
- LEDs 6–7 show actuator command

There are also special LED indications for manual mode and depth zeroing. :contentReference[oaicite:4]{index=4}

## Hardware summary

Main hardware used in the current build:

- SparkFun Pro Micro ESP32-C3
- Blue Robotics Bar02 pressure sensor
- DS1307 RTC
- Servo-driven buoyancy actuator
- NeoPixel Jewel
- Transparent PVC enclosure

The current firmware is written around those devices and uses the ESP32-C3’s Wi-Fi, I2C, and PWM features to run the system. :contentReference[oaicite:5]{index=5}

## Repository structure

This repo is intended to hold the full project, including:

- mechanical design files
- electronics design files
- firmware
- test notes
- project documentation
- photos and supporting media

## Current firmware notes

The current firmware supports:

- onboard depth zeroing before mission start
- actuator priming at startup
- PID depth control
- mission state machine control
- hold timing that resets if the profiler leaves tolerance
- sample logging in RAM
- mission and PID updates from the client
- manual actuator override
- onboard LED status feedback

The current actuator range is 500 to 2500 microseconds, with 1500 microseconds as neutral. The firmware also includes a manual mode and adjustable mission parameters through the web page. :contentReference[oaicite:6]{index=6}

## Project goals

This project was built to meet the MATE competition requirements while also giving us a platform that was easy to test, easy to tune, and cheap enough to build without overcomplicating the design.

A big part of the project was making something that could be adjusted quickly in testing. That is why so much of the software was focused on live tuning, telemetry, and usability instead of just running one fixed script.

## Status

This project is functional and actively being tested and revised. The hardware, software, and control system have all gone through multiple revisions, especially in the buoyancy engine and controls.

## More documentation

More detailed project writeups, photos, and supporting files are included elsewhere in this repository and in the linked project documentation.