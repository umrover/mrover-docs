---
title: "USB Passthrough for UTM"
---
### Are you running Ubuntu through UTM? Trying to access a USB device in your VM? Struggling to get it working?

Try this simple 2-step process that helped me fixed my problem! This is what I did - 

1. **USB 2.0 or USB 3.0?** Check the USB connector - if the block inside is black, it's USB 2.0. If it's blue, it's USB 3.0. Then, you're going to want to set up USB sharing in UTM for your VM to match up with that. Refer to the GIF below for specifics on this
 &nbsp;<img src="https://user-images.githubusercontent.com/31768361/215346181-0c0a2d85-32a4-4b88-a701-ac4b076ad1f1.gif" alt="USB sharing" width="700">
2. Select the USB device in UTM. Boot up your VM, go to the top right menu (of UTM, not your VM), click on the USB icon, & select your USB device. Look at the GIF below for what this looks like

 &nbsp;<img src="https://user-images.githubusercontent.com/31768361/215346650-2b80e16a-bb6e-49fb-ab37-66e3072a0555.gif" alt="Select USB device" width="500">

And that should be it! You should be able to see your device with the `dmesg | grep tty` command.

p.s. If this didn't work/ if you've found a different way of getting USB pass through to work for you, let me know! Or just update this page directly.