---
title: "Raman"
---
## Overview

We are using Raman spectroscopy to detect substances in samples for science detection tasks. This document has been made to help anyone working with Raman.

## Software

The software and lots of the instructions and information provided in this document are in reference to the content found on [TCD1304.wordpress](https://tcd1304.wordpress.com/). There will be links to specific locations on this website throughout, to help show where information is found in case extra clarity is  needed.

## How Does It Currently Work?

The CCD of the Raman setup is connected to a STMF401RE Nucleo using software from [TCD1304.wordpress](https://tcd1304.wordpress.com/). This Nucleo can be connected to a PC, jetson, or raspberry pi and with a python application, a GUI can be displayed showing the wavelengths it is reading.

## What is Raman?

Raman is a spectrometer setup we use on the rover. A spectrometer can conduct spectroscopy, the study of the absorption and emission of light and other radiation through matter. Essentially, this means we can shine a laser at a sample and based on the light that is not absorbed, we can detect if certain elements are present. The CCD is a sensor and is the portion of the Raman setup that collects light. This is what is connected to the STMF401RE Nucleo. The goal of the Raman is to detect kerogen and carotenoids’ spectral lines which are detected by the CCD. Kerogen is fossilized organic matter and carotenoids are cyanobacteria. Light is shone onto a sample and the light reflected is captured by the CCD. The reflected wavelengths are compared to known values so that we can characterize compounds, for example Kerogen has wavelengths of 713 nm and 726 nm while carotenoids are 695 nm, 703 nm, and 721nm.

## The CCD

As mentioned above, the CCD is the sensor that collects the light and relays information to the STMF401RE. We are using the TCD1304, a high pixel CCD sensor used in spectrometers. The nucleo must be set up in order to receive information from the CCD that can then be read by a computer (PC or Jetson or Raspberry PI) through a command line interface or a graphical user interface.

## Connecting the CCD to the STM32F401 Nucleo

Wire the two as follows:
* fM connects to PB0
* SH connects to PA1
* ICG connects to PA0
* Output connects to PC0
* GND to GND
* Power
    * Connect +5V to +5V if you have a TCD1304-PCB with a LDO in place
    * If voltage is not regulated by the CCD, use 3.3V or 5V (do this)

## General Setup (Needed for both data collection methods)

* Download TCD1304 driver firmware (UART) from [Downloads | TCD1304.wordpress](https://tcd1304.wordpress.com/downloads/)
* Unzip the downloaded .zip file
* Connect STMF401RE to computer and wait for file explorer browser to open (this will show the storage of the nucleo)
* Open the TCD1304 folder that was unzipped and look for the .bin file
* Drag the .bin into the main folder of the nucleo (basically, do not be in any of the folders
* The .bin should disappear

## Setup for Python GUI

Complete General Setup and en* sure the Nucleo is still plugged in
* Download pyCCDGUI (python) from [Downloads | TCD1304.wordpress](https://tcd1304.wordpress.com/downloads/)
* On a windows computer
* Run “python3 pyCCDGUI.py” from your computer (you need to be in the same directory) and it should open up a GUI
* For COM-device, use COMX where X is a random number (4 worked for me)
* You will know when the device is found
* Press Collect and data will be collected and displayed on display
* On a Linux computer (Jetson, etc.)
* User must have read/write permissions to /dev/ttyACM0
* To do this, you type $ sudo chmod 666 /dev/ttyACM0
* To find current permissions of a file, do $ stat -c %a /dev/ttyACM0
* For Mac, I found I needed to find my own port name by doing ls /dev/tty.usb* and then using what it printed rather than /dev/ttyACM0
* Run “python3 pyCCDGUI.py” from the computer (you need to be in the same directory) and it should open up a GUI
* For COM-device, use /dev/ttyACM0 
* You will know when the device is found
* Press Collect and data will be collected and displayed on your display (Might need to connect Jetson to a TV with HDMI)
* For accessible data, check out Setup for Command Line Interface, since it outputs data to files

## Socat Transmission

On the Jetson:

Need to add the following to the 99-usb-serial.rules: 
```SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", SYMLINK+="raman"```
This makes it so that /dev/raman corresponds to the STM32F401

Make sure to run ```sudo apt install socat```

On linux device with the STM32F401 Nucleo plugged in to USB, the following needs to be ran: 
```/usr/bin/socat tcp-listen:8888,reuseaddr,fork file:/dev/raman,nonblock,waitlock=/var/run/tty0.lock,b115200,raw,echo=0```

You might need to run the following on the basestation:

```
socat tcp:10.0.0.2:8888 file:/dev/raman,nonblock,waitlock=/var/run/tty0.lock,b115200,raw,echo=0
```


