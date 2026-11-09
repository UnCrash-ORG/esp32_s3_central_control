# Robot Central Controller

A robot central controller developed for a school robotics competition. The repository contains ESP32-S3 firmware and hardware design files. Built with PlatformIO and ESP-IDF, the firmware connects motion actuators, an inertial sensor, and a serial communication interface.

> This project is under development. The control task, some serial command handlers, and motor parameters are not yet complete. Further hardware and firmware integration testing is needed.

## Project overview

- **Motion control:** initialization and components for four DC motors with encoders, two stepper motors, and four servos.
- **Inertial sensing:** GY-521 (MPU6050) sensor data acquisition and a Mahony AHRS algorithm component.
- **Serial communication:** message definitions for IMU requests, velocity settings, stepper motors, and servos. Some handlers are still under development.
- **Hardware design:** central controller schematics and PCB project files in `lceda/`.

## Development environment

- [PlatformIO Core](https://platformio.org/install/cli) or an IDE with PlatformIO support
- Target board: `esp32-s3-devkitc-1`
- Framework: ESP-IDF
- Flash configuration: 8 MB; see `partitions.csv` for the partition table

Run these commands from the project root:

```bash
# Build the firmware
pio run

# Upload the firmware (connect the board and select the correct serial port)
pio run --target upload

# Run static analysis
pio check
```

To specify the serial port, add `--upload-port <port>` to the upload command. See [`main/include/pin_config.h`](main/include/pin_config.h) for pin assignments and refer to the schematics when connecting hardware.

## Project structure

```text
.
├── .github/workflows/       # Static analysis and firmware build workflows
├── lceda/                   # EasyEDA schematics, PCB, and project files
├── main/
│   ├── components/          # Motor, sensor, and attitude estimation components
│   ├── include/             # Pin, task, and communication definitions
│   └── src/                 # Entry point, control, and serial tasks
├── test/                    # PlatformIO test directory
├── partitions.csv           # Flash partition table
└── platformio.ini           # PlatformIO project configuration
```

## Continuous integration

GitHub Actions runs PlatformIO static analysis and builds the firmware on pushes to `main` and pull requests targeting `main`. Firmware artifacts are available from successful workflow runs. There are currently no automated test cases.
