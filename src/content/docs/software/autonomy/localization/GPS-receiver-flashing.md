---
title: "How to flash the GPS receiver"
---
# Introduction

You shouldn't have to flash the GPS receivers with new config options often. However, for when you do, in the most dire of times, this tutorial will be here for you. So far we only have instructions for the basestation receiver, but I'm sure with the documentation I linked and your determined efforts, you can figure out how to set up the rover GPS receiver also.

## Overview

When flashing a GPS receiver, you have two main options: pygpsclient and u center. U center is the official software, but only runs on windows, so it's lame. 

The documentation for the config options mentioned will be in the References section at the bottom of this page.

## Pygpsclient

First, download the software. You're a capable young individual, so I'm sure you can figure out how to do this with the help of google. I recommend doing it in a venv to avoid any possible issues.

Then, run the software. You might need to install tkinter, which again, you are a talented Mrover member following this tutorial, so figure it out.

Also, if it's not already, plug in the GPS receiver. How do you expect to flash it if it's not even plugged into your computer, you silly goose.

Now follow the following steps:

* change rate (in the top right) till "u-blox GNSS reciever" shows up on ttyACM0, probably at 57600 baud
* hit USB/UART to connect
* should show connected in green in bottom left, messages should appear in the middle area

goto the UBX config. For these you can use CFG-VALGET to read the value, CFG-VALSET to write. You'll need to write the FLASH (so it doesn't get wiped on powercycle), and RAM (to update it immediately)

First we need to set CFG_TMODE settings
* CFG_TMODE_MODE - should be 1 for survey in, as shown on pg 268 of the docs
* CFG_TMODE_SVIN_ACC_LIMIT - should be 10000 for 1m (yeah the units are like tenth of millimeter, yeah it's weird)
* CFG_TMODE_SVIN_MIN_DUR - default is 60s, should be fine to leave it that way. this defines the minimum time for a survey-in
* All the rest in CFG_TMODE can be ignored, as they're all for FIXED mode
  * If you want to do FIXED mode, as we've considered in the past to cut down on basestation survey-in time for comp time crunch, set these appropriately
    * i believe in you, you can figure it out yourself, i will not be helping you in this tutorial
Now we shall set which RTCM messages we will broadcast, so go to CFG_MSGOUT_RTCM and set the following ([source](https://www.ardusimple.com/how-to-configure-ublox-zed-f9p/), under Configuration: Base)
* CFG_MSGOUT_RTCM_3x_TYPE1005_USB = 1
* CFG_MSGOUT_RTCM_3x_TYPE1074_USB = 1
* CFG_MSGOUT_RTCM_3x_TYPE1084_USB = 1
* CFG_MSGOUT_RTCM_3x_TYPE1094_USB = 1
* CFG_MSGOUT_RTCM_3x_TYPE1124_USB = 1 // this one is not in the source linked, but it's for the BeiDou constellation of satellites and should be good bc theres a lot of BeiDou satellites
* CFG_MSGOUT_RTCM_3x_TYPE1230_USB = 1
Other settings of note
* CFG-MSGOUT-NMEA_ID_GGA_USB - this, when set to 0, will disable NMEA outputs so you just get RTCM corrections. Idk if this is helpful but it might decrease bandwidth used? so set it to 0 at your own risk because i don't think the NMEA messages are important but i'm not sure
* CFG-RATE-MEAS - This controls the rate of publishing messages. 1000 (1000ms=1hz) should be good for basestation, BUT something closer to 50 (50ms=20hz) is preferred for the rover gps antenna

## U-center

* i donno i've never done it good luck
* just kidding (mostly), you can probably mostly set the same stuff as above but the UI might be a bit different idk. Also see the references section below for a guide.

## References

[config options reference](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf) - this document has all the config options listed above, what different values mean, and what they do. For any questions you may have, like "How do I know that CFG_TMODE_MODE is 1 for survey in?" (pg 268) or "What's the unit for CFG_TMODE_SVIN_ACC_LIMIT?" (also pg 268) or "What is RTCM message type 1005?" (pg 202)
[RTCM messages source](https://www.ardusimple.com/how-to-configure-ublox-zed-f9p/) - I took some tips from here on which RTCM message types to use. It also seems to have some instructions for using u-center.


Good luck!
