# Variable Power Source

An essential tool in an electronics workshop is a Variable Power Source. This must regulate current and voltage delivered in addition to having protections against short circuits.

This Variable Power Supply is built by modifying an old PC ATX power supply and based on a tutorial made by Cuban electronics [here](http://cubaelectronica.blogspot.com/2017/06/hacer-fuente-atx-pc-variable-desde-1-28.html) where the modifications that need to be made to the PC ATX Power Supply are explained.

>### _**Here the microcontroller control board is made to achieve better control precision. It also provides a real-time current, voltage and power measurement.**_

### _**A complete tutorial for make this Variable Power Supply is now available!!!**_
### _**https://www.sysadminsdecuba.com/2021/09/convierta-una-fuente-de-computadora-atx-en-una-fuente-variable-0-30v-0-15a/**_

## Features

* Complete range ( from 0V to 30V and 0A to 15A ) control with a maximum precision for a wide range.
* Fast short-circuit protection.
* Minimum modifications made to ATX Power Supply.
* Nice and clear interface.
* ~~5V (of Standby supply) for USB plug~~ (Not supported yet).
* small design (all into ATX chassis) by choice.

## What do you need ?

* Bluepill (STM32F103C8T6 microcontroller based board).
* OLED 128x64 0.96"(SSD1306 based)  ~~display or LCD 16x2 Display~~(Actually not supported).
* A rotary encoder with central Button.
* LM324 IC.
* BD139 Transistor.
* Some others common components (Resistors and Capacitors, LEDs, Connectors, a little switch, etc).

## Softwares Used

* STM32CubeIDE 1.3.0 (Firmware and Debug).
* KiCAD (Schematic and PCB).
* NI Multisim 14.0 (Simulations).


## **Important Note**

>PCB layout is made to do at home (DIY). They are not optimized to manufacture with a PCB manufacturing company like JLCPCB or others.

## Preview

![PCB](Schematic_and_PCB/pcb_preview_1.jpg)
![PCB](Schematic_and_PCB/pcb_preview_2.jpg)


