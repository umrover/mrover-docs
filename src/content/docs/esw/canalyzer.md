---
title: "CANalyzer"
---
# CANalyzer

CANalyzer will be a custom CAN bus analyzer tool device. The end-goal is to have a fully embedded
device that can be used to monitor and log CAN bus traffic. Use a Raspberry Pi with an FDCANUSB or PiCAN FD HAT to receive and handle any traffic on a CAN bus using SocketCAN.

## Project Goals

- **Real-time Monitoring:** Display CAN bus messages in a structured and easy-to-read format
- **Custom CAN Message Handling:** Implement support for our team's custom CAN message format
- **Filter and Search:** Implement filters to allow users to focus on specific CAN IDs or data patterns using custom configurations provided in a .yaml file.
- **Logging:** Enable saving of CAN bus data to a file (.asc or .log) for later analysis
- **Replaying Logs:** Ensure that log files are properly formatted in order to replay logs for testing using the [can-utils](https://manpages.debian.org/testing/can-utils/candump.1.en.html) Linux package or something similar.
- **Other Thoughts:** Add capability for adjusting CAN message format - extended/standard ID, BRS on/off, etc.
