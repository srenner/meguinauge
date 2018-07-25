# meguinauge

**MEG**asquirt + ard**UIN**o + g**AUGE** = meguinauge. 20x4 character LCD gauge for [Megasquirt](http://megasquirt.info/).

## Overview
* Provides a vehicle gauge that can display 2 - 8 different engine parameters at a time.
* Designed for a 1980s-1990s aesthetic. For modern looking gauges, check out tablet and Pi based solutions.

## Hardware details (used in early development)
* [Arduino UNO](https://www.arduino.cc/en/main/arduinoBoardUno) or compatible board
* [CAN-BUS shield](https://www.sparkfun.com/products/13262)
* [20x4 LCD display](https://www.adafruit.com/products/499)
* Red LED for error light (optional)
* Button to select gauge (optional)
* Button to select mode (optional)

## Hardware details (to be installed in car)
* [Arduino-compatible AST-CAN485 Dev Board](https://www.sparkfun.com/products/14483)
* 20x4 LCD display with serial communication
* Red LED for error light
* Button to select gauge
* Button to select mode

## Extra Arduino dependencies (used in early development)
* [CAN-BUS library](https://github.com/Seeed-Studio/CAN_BUS_Shield) from Seeed-Studio.

## Megasquirt details
* Tested using Megasquirt-3 with firmware version 1.5.0.
* Uses Megasquirt's "Simplified Dash Broadcasting" as described in [this PDF](http://www.msextra.com/doc/pdf/Megasquirt_CAN_Broadcast.pdf).


