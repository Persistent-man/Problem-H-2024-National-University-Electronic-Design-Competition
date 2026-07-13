底层借鉴B站博主“我的代码没问题”||给我加学分hbwlxy-2025118228

# STM32F103C8T6 Intelligent Tracking Car

This repository contains the firmware and supporting material for a two-wheel differential-drive tracking car. The project is built around an STM32F103C8T6, an 8-channel grayscale tracking sensor, a TB6612 motor driver, dual encoders, and an MPU6050 IMU.

The runnable Keil project is in `car_example/user/Project.uvprojx`.

## Features

- 8-channel grayscale line tracking for straight and curved paths.
- Dual-motor speed closed loop using encoder feedback and PID control.
- MPU6050 gyroscope yaw integration for heading correction and in-place turns.
- Four selectable route modes, selected with a button before starting.
- OLED display for the selected mode, route state, yaw angle, and encoder counts.
- Two-second delayed start after pressing the start button.

## Hardware

| Part | Model or type | Purpose |
| --- | --- | --- |
| Main controller | STM32F103C8T6 | Reads sensors and controls the car |
| Motor driver | TB6612FNG | Drives the left and right DC motors |
| Motors | Two DC geared motors with encoders | Differential drive and distance feedback |
| Tracking sensor | 8-channel digital grayscale module | Detects the black line |
| IMU | MPU6050 | Provides angular velocity for yaw estimation |
| Display | 0.96-inch I2C OLED (SSD1306 compatible) | Shows status and debugging data |
| Buttons | Two momentary buttons | Mode selection and start |
| Programmer | ST-Link | Downloads firmware to the STM32 |

## Pin and Wiring Map

All modules must share a common ground with the STM32. The STM32 GPIO pins are 3.3 V logic. Do not connect a 5 V sensor output directly to an STM32 input unless the module is confirmed to output 3.3 V logic or a level shifter is used.

### TB6612FNG motor driver

| STM32 pin | TB6612 signal | Description |
| --- | --- | --- |
| PA0 / TIM2_CH1 | PWMA | Left motor PWM speed control |
| PA6 | AIN1 | Left motor direction input 1 |
| PA7 | AIN2 | Left motor direction input 2 |
| PA1 / TIM2_CH2 | PWMB | Right motor PWM speed control |
| PB0 | BIN1 | Right motor direction input 1 |
| PB1 | BIN2 | Right motor direction input 2 |
| 3.3 V | VCC | TB6612 logic supply |
| Motor battery positive | VM | Motor supply; follow the driver and motor voltage rating |
| GND | GND | Common ground |
| 3.3 V or a GPIO held high | STBY | Driver enable; STBY must be high for the motors to run |

Connect the left motor to `A01/A02`, and the right motor to `B01/B02`. If forward and reverse are swapped, exchange that motor's two wires or reverse its direction definition in `code/motor.c`.

### Motor encoders

| STM32 pin | Encoder signal | Description |
| --- | --- | --- |
| PA2 / EXTI2 | Left encoder A | Falling-edge pulse count |
| PA3 | Left encoder B | Direction judgment |
| PA4 / EXTI4 | Right encoder A | Falling-edge pulse count |
| PA5 | Right encoder B | Direction judgment |

The code uses quadrature decoding. The calibration value is currently `12.5 pulses/cm`; adjust `Pulse_Per_cm` in `code/route.c` to match the real wheel and encoder.

### 8-channel grayscale tracking module

| Sensor channel | STM32 pin |
| --- | --- |
| D1 | PB12 |
| D2 | PB13 |
| D3 | PB14 |
| D4 | PB15 |
| D5 | PA8 |
| D6 | PC13 |
| D7 | PC14 |
| D8 | PC15 |

The current tracking code treats a low level (`0`) as black-line detection. The sensor layout is D1-D8 from left to right. If the module has an inverted output, reverse the judgment in `code/gray_track.c`.

### MPU6050

The MPU6050 uses software I2C.

| STM32 pin | MPU6050 pin |
| --- | --- |
| PB10 | SCL |
| PB11 | SDA |
| 3.3 V | VCC |
| GND | GND |

The program initializes the MPU6050 at startup, waits briefly, calibrates the Z-axis gyroscope bias, then resets the current heading to zero. Keep the car stationary during power-up calibration.

### OLED display

The OLED uses a separate software I2C bus.

| STM32 pin | OLED pin |
| --- | --- |
| PB8 | SCL |
| PB9 | SDA |
| 3.3 V | VCC |
| GND | GND |

### Buttons

| STM32 pin | Function |
| --- | --- |
| PA11 | Mode selection; cycles through modes 1-4 |
| PA10 | Start; starts the selected mode after a two-second delay |

The source detects a rising edge. Wire each button so that the pin is normally low and becomes high when pressed, with appropriate pull-down resistors or a module that provides them.

## Control Logic

`TIM3_IRQHandler()` runs the main control task every 10 ms:

1. Reads encoder pulse changes and updates traveled distance.
2. Updates the route state machine.
3. Runs the two motor speed PID loops.
4. Updates PWM and motor direction outputs.

`gray_track.c` calculates a line position error from the eight grayscale inputs. It applies PD steering, performs stronger corrective turns on edge detection, and searches in the previous direction when the line is lost.

`route.c` provides four route modes. Distances, speeds, and target angles are configuration constants at the top of that file. These values are specific to the test track and should be recalibrated when the chassis, encoder, or track changes.

## Build and Download

1. Install Keil MDK and the STM32F1 device pack.
2. Open `car_example/user/Project.uvprojx` with Keil uVision.
3. Select target `STM32F103C8` and build the project.
4. Connect ST-Link to the STM32 `SWDIO`, `SWCLK`, `3.3V`, and `GND` pins.
5. Download the program with Keil.
6. Place the car on a stable surface, power it on, and wait for MPU6050 calibration to finish.
7. Press the mode button to choose mode 1-4, then press start. The car starts after about two seconds.

## Project Layout

```text
car_example/
  code/       Application modules: motor, PID, tracking, filtering, route state machine
  ml_libs/    STM32 peripheral and sensor drivers
  sys/        STM32F1 startup and system files
  user/       Keil project, main program, and interrupt handlers
template/     STM32F1 blank project template
资料/          Datasheets and reference manuals
源码/          Independently organized PID, tracking, and filter source files
```

## Important Notes

- `ml_hmc5883l` and UART drivers are included in the source tree, but the current `main.c` leaves their initialization commented out. They are not required for the current car operation.
- Keep motor power separate from the STM32 3.3 V supply. Only connect their grounds together.
- Check motor polarity before allowing the car to run freely; lifting the wheels during the first test is recommended.
- This repository should contain source code and documentation, not compiled Keil output, debug logs, drivers, or third-party executable tools.

## Acknowledgments

The original project materials acknowledge e-core Studio, Zhufei Technology, 正点原子, and 江协科技 for reference material and driver implementation ideas. Preserve applicable upstream notices when redistributing this project.
