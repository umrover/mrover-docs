---
title: "Voice Detection"
---
**"Tags"**: Python, C++, AI/ML, Voice Recognition Models, AI on Edge Devices

**TL;DR**: During the Autonomy Mission, the astronaut will need to issue a series of commands (such as "follow", "come", "fetch", etc) to the rover. While this might be done using gestures, we will also implement voice recognition of these words. The full list of commands can be found in the [URC rules](https://urc.marssociety.org/home/requirements-guidelines) on the website. Since the command list is so small, we will likely run the voice recognition off of the Raspberry Pi 4 attached to the astronaut. This project entails training a voice recognition AI model that can run off of the Pi. The key to this project is the tradeoff between how accurate/powerful the model is and the available compute and power on the Pi 4. Inference should be low-cost (for example, using LiteRT) so that radio communication and battery life are not severely impaired for the astronaut. 