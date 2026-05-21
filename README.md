# Pong Arcade Console
A bare-metal C implementation for ATmega328P.

## Hardware Setup
- **Microcontroller:** ATmega328P (Arduino Uno)
- **Display:** 16x16 LED Matrix (MAX7219)
- **Input:** 2x Analog Joysticks
- **User Interface:** 2x 1602 LCDs (I2C)

## Project Overview
This project manages game states, SPI matrix rendering, and I2C communication using direct register access—no Arduino libraries.