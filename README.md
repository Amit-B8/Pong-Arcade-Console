# Pong Arcade Console
A bare-metal C implementation for the ATmega328P microcontroller, featuring a 4-player tournament mode, SPI LED matrix display, and dual I2C LCD user interfaces.

---

## Hardware Setup & Pinout
* **Microcontroller:** ATmega328P (Arduino Uno board used as a breakout)
* **Display:** 16x16 LED Matrix (driven via MAX7219 / SPI)
* **Input:** Dual Analog Joysticks (connected to ADC channels)
* **User Interface:** Two 1602 LCDs communicating simultaneously over a shared I2C (TWI) bus

---

## Project Overview
This project completely bypasses standard Arduino libraries, utilizing **bare-metal C** and direct register manipulation. It features:
* A non-blocking Finite State Machine (FSM) driven by a 1ms hardware timer interrupt.
* Real-time paddle and ball physics.
* Concurrent protocol communication (SPI and I2C/TWI).

---

## How to Build and Run

### Prerequisites
Make sure you have the AVR toolchain installed on your system:
* `avr-gcc`
* `avrdude`
* `make` (optional, or use direct commands)

### Compilation & Flashing
1. Clone the repository to your local machine:
   ```bash
   git clone [https://github.com/Amit-B8/Pong-Arcade-Console.git](https://github.com/Amit-B8/Pong-Arcade-Console.git)
   cd Pong-Arcade-Console