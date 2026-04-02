# EN2533 Robot Design and Competition

I received an A pass for this module.

Arduino/PlatformIO codebase for our competition robot (ATmega2560) with modular task-based behaviors.

## Robot Photos

Robot images:

<img src="assets/images/WhatsApp%20Image%202026-04-03%20at%2000.06.09.jpeg" alt="Robot Photo 1" width="280" />
<img src="assets/images/WhatsApp%20Image%202026-04-03%20at%2000.06.09%20(1).jpeg" alt="Robot Photo 2" width="280" />
<img src="assets/images/WhatsApp%20Image%202026-04-03%20at%2000.06.10.jpeg" alt="Robot Photo 3" width="280" />
<img src="assets/images/WhatsApp%20Image%202026-04-03%20at%2000.06.11.jpeg" alt="Robot Photo 4" width="280" />

## Hardware/Platform

- Board: Arduino Mega 2560 (`megaatmega2560`)
- Framework: Arduino
- Build system: PlatformIO
- Core dependencies:
  - `pololu/QTRSensors`
  - `Adafruit_VL53L0X`

## What I Have Implemented

### Core Robot Modules

- Motor driver abstraction (`MotorControl`)
- Encoder-based distance control (`MoveController`)
- PID turn control with encoders (`TurnController`)
- QTR line sensor wrapper and calibration (`LineSensors`)
- PID line following (`LineFollower`)
- Dual ToF sensor support (`ToFSensor`)
- Ultrasonic front distance sensor (`UltrasonicSensor`)
- RGB LED and color sensing modules (`RGBLED`, `ColorSensor`)
- Circle-following behavior using dual ToF (`CircleFollower`)
- DIP-switch based task selection (`GetTaskNumber`)

### Task Implementations

- `Task1`: Grid solver logic with color detection, obstacle detection, and encoder-based moves.
- `Task2_1`: Line following + intersection handling + movement and turn sequence.
- `Task2_2`: Continued line logic with final maneuver sequence and recovery.
- `RampTask`: Ramp handling task module.
- `Task4_1`: Line-following with obstacle trigger to circle-follow mode.
- `Task5_Arrow`: Arrow-track behavior with IR + line sensors and recovery logic.

### Main Runtime Integration

Current `src/main.cpp` contains:

- Full robot/module initialization
- Sensor calibration and startup pipeline
- Sequential task runner with DIP-selected start task
- Integrated flow for Task 2, Ramp task, and Task 4

Note: `runTask1()` in `main.cpp` is currently a placeholder in the runtime switch, while full `Task1` implementation exists in `src/Task1.cpp`.

## Project Structure

- `src/` - Main application and task implementations
- `include/` - Headers and module interfaces
- `examples/` - Standalone test sketches (`IR_Calibration`, `LineFollowing`, `MotorControlTest`)
- `test/` - Unit test scaffold (currently template)

## Build and Upload

```bash
pio run
pio run -t upload
pio device monitor
```

## Recent Repository Update

Latest update includes:

- Added task source/header files for Task 1 and Task 5
- Updated Task 2 modules and main task flow
- Added build output log snapshot (`build_output.txt`)
- Synced repository to GitHub remote under this account
