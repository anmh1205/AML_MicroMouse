# AML MicroMouse

An autonomous Micromouse robot built around an **STM32F407VET6** microcontroller and VL53L0X laser distance sensors, with firmware, PCB design files, maze-solving experiments, and simulator resources.

[![Watch the AML MicroMouse competition video](https://img.youtube.com/vi/a5xwwyJaq_M/hqdefault.jpg)](https://www.youtube.com/watch?v=a5xwwyJaq_M)

*Competition footage — [watch on YouTube](https://www.youtube.com/watch?v=a5xwwyJaq_M).*

## Project overview

In a Micromouse competition, a robot must explore an initially unknown maze, recognize walls, determine a route toward the goal, and execute that route accurately. This project brings together embedded sensing, motion control, route planning, custom electronics, and simulation in a single engineering workspace.

The repository includes STM32 firmware workspaces and supporting experiments rather than a single installation package. It also retains the hardware and simulation materials used during development.

## Technical scope

| Area | Repository contents |
| --- | --- |
| Controller | STM32F407VET6-based firmware project |
| Wall sensing | VL53L0X laser-ranging sensors |
| Firmware toolchain | STM32CubeIDE-compatible project metadata, STM32CubeMX `.ioc` configuration, and build files |
| Embedded software | Sensor acquisition, control code, and maze-navigation experiments |
| Simulation | Micromouse simulator materials and C examples |
| Electronics | PCB design assets and bill of materials |

## Exploring and building the firmware

Start with the STM32 project under `1. MicroMouse Code/0. MicroMouse - STM32F407VET6/PCB_V1`. That directory contains `.project`, `.cproject`, `PCB_V1.ioc`, source code, startup assembly, linker scripts, and other toolchain files.

For a first build:

1. Install an STM32 development toolchain compatible with the included project, such as STM32CubeIDE.
2. Import the `PCB_V1` project from the existing-project workflow.
3. Review the MCU configuration, compiler settings, external libraries, and selected build configuration.
4. Build for the intended hardware. Flash only after confirming connections, debugger settings, and motor safety.

Some directories contain generated `Debug/` or `build/` artifacts from earlier development. These are historical outputs, not evidence that the current project has been revalidated with recent toolchain versions.

## Repository guide

- [Examples](0.%20Example/) — source material and technical experiments.
- [Firmware and controller projects](1.%20MicroMouse%20Code/) — STM32 firmware, ESP32 remote experiments, and C tests.
- [PCB design](2.%20MicroMouse%20PCB/) — electronics project materials.
- [Mechanical drawings](4.%20Drawing/) — design and mechanical references.
- [Simulation resources](5.%20Simulator/) — Micromouse simulator and example code.
- [Bill of materials](Bill.xlsx) — component inventory reference.

## Development context

The repository captures the engineering process behind a competition robot, including intermediate experiments and physical design files. Several firmware and simulator resources coexist; they should not be assumed to represent one unified production release.

No repository-wide license is currently declared. Check the terms of bundled libraries and third-party simulator materials before reuse or redistribution.
