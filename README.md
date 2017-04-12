# meguinauge

**MEG**asquirt + ard**UIN**o + g**AUGE** = meguinauge. 20x4 character LCD gauge for [Megasquirt](http://megasquirt.info/).

## Overview
* Provides a vehicle gauge that can display 2 - 8 different engine parameters at a time.
* Designed for a mid-'80s aesthetic. For modern looking gauges, check out tablet and Pi based solutions.

## Hardware details
* [Arduino UNO](https://www.arduino.cc/en/main/arduinoBoardUno) or compatible board
* [CAN-BUS shield](https://www.sparkfun.com/products/13262)
* [20x4 LCD display](https://www.adafruit.com/products/499)
* Red LED for error light (optional)
* Button to select gauge (optional)
* Button to select mode (optional)

## Extra Arduino dependencies
* [CAN-BUS library](https://github.com/Seeed-Studio/CAN_BUS_Shield) from Seeed-Studio.

## Megasquirt details
* Tested using Megasquirt-3 with firmware version 1.5.0.
* Uses Megasquirt's "Simplified Dash Broadcasting" as described in [this PDF](http://www.msextra.com/doc/pdf/Megasquirt_CAN_Broadcast.pdf).


