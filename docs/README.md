# Thermal Turret

This is a project developed as a proof of concept to a thermometer that can recognize human faces and direct a thermal laser at the forehead to measure the temperature of the person. This was initially built during the 2019 Covid19 Pandemic but has since been replaced by better systems and designs into 2020 and 2021.

All links to important documentation are located in [development-references](/docs/development-references.md)

## Basic Hardware Diagram

A more  thorough wiring sheet with descriptions and pins can be found in the [wiring](/docs/wiring.md) page

![image unavailable](/docs/img/hardware-diagram.png)

## Components and Codebase

### Infrared Thermometer
The thermal turret uses off the shelf consumer parts including a SEN-09570 infrared thermometer from sparkfun in a 3d printed case which can be foung in the [/cad](/cad) folder. The basic functionality can be seen by using the arduino project in the [/misc](/misc) folder. It also gives all information for locations of memory and important communication addresses for the temperature sensor.

There is also a TMP36 touch sensor for more fine tuned temperature readings in this specific prototype. When building the final prototype this is expected to be removed. It is added at this moment to compare between the SEN-09570 and itself for accuracy testing of the SEN-09570.

### Microcontroller
The microcontroller on this device is an STM32F401RETX nucleo development board. You can use the standard CubeMX IDE to compile and build the project on the nucleo seamlessly. You can follow the [wiring diagram](/docs/wiring.md) to see how to link all parts of the project. 

### Computer Vision

The computer vision device is a Jetson Nano. The developer kit which aided in cooling and running the jetson at high speeds. One can replace this with a suitable heat sink and fan combination. It uses OpenCV and a precompiled neural network for recognizing faces. There is a crontab to run the openCV script on startup as to not need setup beforehand. There is also a display linked to the Jetson Nano to display the view to the user. The Jetson Nano links with the Nucleo board by a parallel bus.

### Serial LCD

There is a serial LCD with RGB backlight linked with the project to display the temperature and other information. The reason for the RGB backlight is so that a user can see at a glance wether they are allowed to pass. The LCD will go green for safe temperatures, yellow if it wants to test you with the touch sensor and red if your temperature is obviously too high. 

## Basic Code Flow

![image unavailable](/docs/img/code-flowchart.png)