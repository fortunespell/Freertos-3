# STM32F767ZI Project with FreeRTOS

## Table of Contents
- [Project Overview](#project-overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Key Functions](#key-functions)
- [Hardware Requirements](#hardware-requirements)
- [GPIO Pin Configuration](#gpio-pin-configuration)
- [Use Cases](#use-cases)
- [Getting Started](#getting-started)
- [License](#license)

## Project Overview
This project involves an embedded system application developed using the STM32F767 microcontroller and FreeRTOS. It includes tasks for controlling LEDs, interacting with an ultrasonic sensor, and using an I2C-based OLED display. The project is structured with multiple RTOS tasks for concurrent operations like blinking LEDs and measuring distance using the ultrasonic sensor.

![image](https://github.com/user-attachments/assets/ffc0d4f7-e600-4d00-a56c-3ae97132387c)

## Features
- **LED Control**: Multi-threaded blinking of red, green, and blue LEDs with varied delays.
- **Ultrasonic Distance Measurement**: The system measures the distance using an ultrasonic sensor and toggles an LED based on proximity.
- **OLED Display**: Displays messages and shapes using an SSD1306 I2C display driver.
- **Button Interaction**: Detects button presses to alter LED behavior.

## Project Structure
- `main.c`: Main program file that sets up the system and implements RTOS tasks for handling LEDs, distance measurement, and OLED display.
- **FreeRTOS** is used to manage task scheduling.
- Several peripherals are initialized, such as GPIO, I2C, and TIM1 for sensor input and control.

## Key Functions
- **LED Tasks**:
  - `StartTask01`: Toggles the red LED and displays text and shapes on the OLED display.
  - `StartTask02`: Toggles the blue LED at different speeds.
  - `StartTask03`: Controls the green LED based on ultrasonic sensor distance.

- **Ultrasonic Sensor**: Measures distance using a trigger and echo method. If the distance is less than 10 cm, the green LED lights up.

- **OLED Display**: Text and graphical content are displayed using the SSD1306 driver library. The display includes numbers, lines, rectangles, and other shapes.

## Hardware Requirements
- **STM32F767ZI Nucleo Board**: The microcontroller for running the program.
- **Ultrasonic Sensor (HC-SR04)**: For distance measurement.
- **I2C-based OLED Display (SSD1306)**: For visual output.
- **LEDs**: Red, green, and blue LEDs connected to GPIO pins.
- **Push Button**: To control the state of an LED.

## GPIO Pin Configuration
- **LEDs**: 
  - Red LED: GPIOB Pin 14
  - Green LED: GPIOB Pin 0
  - Blue LED: GPIOB Pin 7
- **Ultrasonic Sensor**: 
  - TRIG Pin: GPIOC Pin 10
  - ECHO Pin: GPIOC Pin 11
- **I2C Display**:
  - SCL: PB8
  - SDA: PB9
- **Button**: GPIOC Pin 13

## Use Cases
1. **Proximity-Based Alert System**  
   The ultrasonic sensor can be used to measure the distance of objects in its path. If an object comes too close (less than 10 cm), the system triggers a visual alert by turning on the green LED. This feature is useful for parking assist systems, automated door systems, and obstacle detection robots.
   
2. **Multi-LED Notification System**  
   The red, blue, and green LEDs can be configured to blink in different patterns to signal different statuses or alerts. For example, the red LED can signal critical alerts, the blue LED can indicate normal operations, and the green LED can act as a proximity indicator.
   
3. **Real-Time Monitoring with OLED Display**  
   The SSD1306 OLED display can be used to show real-time data such as distances measured by the ultrasonic sensor, system status, or any other important metrics. This use case applies to dashboards for industrial applications, home automation, or IoT displays.
   
4. **Button-Controlled Operations**  
   The push button can be used to modify the system’s behavior, such as switching between different LED blinking modes or toggling specific actions based on user input. This is useful for creating interactive devices where the user needs to trigger specific tasks.
   
5. **Interactive Graphical Display**  
   The OLED screen can also be used to display interactive graphical content like shapes (lines, rectangles, circles), and dynamic numbers (e.g., distance measured). This can be expanded to educational kits for demonstrating graphical output on embedded systems or real-time display of sensor values.
   
## Getting Started
1. **Clone the repository**:
    ```bash
    git clone https://github.com/fortunespell/Freertos-3.git
    ```

2. **Open the project** in STM32CubeIDE.

3. **Build and flash** the project onto the STM32 Nucleo board.

4. **Connect the peripherals** (OLED display, LEDs, ultrasonic sensor) as specified in the GPIO configuration.

5. **Run the project** and observe the behavior of the LEDs, OLED display, and ultrasonic sensor.

## License
This project is licensed under the STMicroelectronics software license. Refer to the LICENSE file in the repository for more details.

