# meguinauge

**MEG**asquirt + ard**UIN**o + g**AUGE** = meguinauge. Pronounced "meh-gween-edge." Or however you want. 20x4 character LCD gauge for [MegaSquirt](http://megasquirt.info/).

## Overview
* Provides a vehicle gauge that can display 2 - 8 different engine parameters at a time.
* Designed for a 1980s-1990s aesthetic.

## Hardware details (used in early development)
* [Arduino UNO](https://www.arduino.cc/en/main/arduinoBoardUno) or compatible board
* [CAN-BUS shield](https://www.sparkfun.com/products/13262)
* [20x4 LCD display](https://www.adafruit.com/products/499)
* Red LED
* 2 pushbuttons

## Hardware details (to be installed in car)
* [Arduino-compatible AST-CAN485 Dev Board](https://www.sparkfun.com/products/14483)
* 20x4 LCD display with serial communication
* Red LED
* 2 pushbuttons

## User Interface
Engine parameters will be displayed on a 20x4 LCD display. Each engine parameter will be shown with a short code and a number. For example, "CLT 203.5" means the coolant temperature is 203.5 degrees. The short codes will be familiar to MegaSquirt users. When 2 or 4 gauges are shown on the display, a bar graph will also be shown for each engine parameter.

The Mode Select button cycles through options to have 2, 4, or 8 gauges on the LCD display at a time. The Gauge Select button changes which specific gauges are shown within the selected Mode.

A red LED is illuminated when one or more of the engine parameters goes outside of its predefined range. When all engine parameters return to their normal range, the LED will stay illuminated for a few more moments. The amount of time the LED remains illuminated depends on the amount of time a parameter was out of range.

## MegaSquirt details
* Tested using MegaSquirt-3 with firmware version 1.5.0.
* Uses MegaSquirt's "Simplified Dash Broadcasting" as described in [this PDF](http://www.msextra.com/doc/pdf/Megasquirt_CAN_Broadcast.pdf).
