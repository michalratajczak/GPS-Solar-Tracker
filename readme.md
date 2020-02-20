# Overview
The main goal of the project is to build a rack for photovoltaic panels that will set them at right angles to the Sun to maximize their efficiency.
# Description
Tracking the position of the Sun is based on obtaining geolocation data as well as the date and time, and then on their basis determine its position in the sky.
<br/>
We used:
- STM32F4 Discovery board
- 2 servomechanisms **TowerPro MG-995**
- GPS module **GY-GPS6MV2 U-blox NEO-6m** 
- Battery 
  - Five 18650 3.7V cells connected in parallel
  - Two controllers from power banks to raise the voltage to 5V
- 4 photovoltaic panels **5V 0.2A** connected in parallel
- STEP-UP XL6009E1 inverter that pulls the voltage from panels up to 5V
<br>
Schematic:
  
<img src="https://i.imgur.com/axwYjmQ.png" style="width: 1000px"/>

Videos:
- [normal mode](https://youtu.be/1S9ANhX0Tlg)
- [presentation mode](https://youtu.be/Gq6PUXdUFNc)
  
# Tools
STM32CubeMX **v4.27.0**  
System workbench for STM32 **v2.9**
# How to run
Just build the project and deploy to your device.
# How to compile
Just copy and compile using System Workbench for STM32.
# Future improvements
- 360 servomechanism 
- digital compass to detect device orientation
# Attributions
We used exported spreadsheet formulas to calculate sun position.<br>
https://www.esrl.noaa.gov/gmd/grad/solcalc/calcdetails.html
# License
MIT
# Credits
## Authors
- Michał Ratajczak
- Norbert Pałuczyński
<br>
<br>
<p>The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.
<br/>Supervisor: Adam Bondyra</p>
