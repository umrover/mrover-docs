---
title: "900x Radio"
---

# 900x Radio Documentation

## Links
[Manufacturer Page](https://store.rfdesign.com.au/rfd-900x-modem/)

[Manual](https://files.rfdesign.com.au/Files/documents/RFD900x%20DataSheet%20V1.1.pdf)

[Software Manual](https://files.rfdesign.com.au/Files/documents/RFD900x%20Peer-to-peer%20V3.X%20User%20Manual%20V1.2.pdf)

[Modem Tools](https://files.rfdesign.com.au/tools/)

## First Time Setup Process
There are two options to configure the radio. I will use AT&M commands over a terminal, because I do not have ready access to a Windows laptop.

### AT&M Setup

See the **Sofware Manual** link for more documentation.

1. Connect FDTI-USB cable to modem & laptop
<img width="305" alt="Image Showing FTDI Cable Connection" src="https://github.com/user-attachments/assets/2fbee0f9-293b-4ebf-ad26-987e65d15538">

2. Connect to the Radio

    i. Determine the device COM port

    List all active connections:

    ```
    ls /dev | grep ttyUSB
    ```

    If it is not obvious, try unplugging the modem, running the command, then plugging it back in and rerunning it. There should be a new device, which is the modem. The first time we did this, the device name was *ttyUSB0*.


    ii. Connect Over Serial

    Default Configuration Settings:

    57600 baud rate, no parity, 8 data bits, 1 stop bit

    ```
    screen /dev/<device_name> 57600
    ```

    **Note: If screen is not installed, install it with:**
    ```
    sudo apt install screen
    ````

3. Configure Using AT&M
    Upon successful connection, you should be greeted with a blank screen.

    i. Type `+++` to enter AT mode

    ii. Type `ATI` to list the current firmware version. Ensure it is up to date.

    iii. Type `ATS3?` - This will display the currently set up network ID. It defaults to 25.

    iv. Type `ATS3=10` - This will set the network ID to 10. By using a non-standard value, we reduce the risk of interference if another team is also using this type of radio.

    v. Type `AT&W` to write the current parameters to EEPROM

    vi. Type `ATO` to exit AT mode

4. Ensure Parameters Have Been Implemented

    i. Repeat the above steps for the other modem(s)-- they must all have the same network ID to function correctly

5. Test Connection

    i. Power on the drone by connecting a battery

    ii. Plug the air-side transmitter into the TELEM1 port of the Pixhawk using the provided cable. It should be attached to the bottom row of the transmitter pins as shown below.

<img width="305" alt="Image Showing FTDI Cable Connection" src="https://github.com/user-attachments/assets/3e187f0f-62e6-4059-89cc-6ce70440ae14">

    iii. Plug the ground-side transmitter into the basestation computer as before.

    iv. Once both are power on, you should see a solid green light, and a blinking red light on both modems.

    v. Launch QGroundControl and verify the connection




