---
title: "Installing Ubuntu"
sidebar:
  label: "Installing Ubuntu"
---

We officially support **Ubuntu 24.04 LTS (noble) running natively**, and only that. Dual-booting needs about 50 GB of free space (100 GB on Perception, for the NVIDIA packages).

If you already have Ubuntu 24.04 installed, or you're setting up a Jetson (ships with its own NVIDIA-provided Ubuntu image), skip straight to [Native ROS Installation](/setup/native-install).

## Dual-Booting Ubuntu 24.04 LTS from Windows

1. Check for BitLocker:
   - Right click the Windows icon → Disk Management, and look for "BitLocker encrypted" on your partitions (usually the largest one).
   - If present, [suspend it](https://4sysops.com/archives/disable-bitlocker-on-windows-11/#rtoc-3), or at least [save your recovery key](https://support.microsoft.com/en-us/windows/finding-your-bitlocker-recovery-key-in-windows-6b71ad27-0b89-ea08-f143-056f5ab347d6).
     - **THIS IS IMPORTANT! WE HAD A MEMBER BRICK THEIR LAPTOP BEFORE**
2. In Disk Management, shrink a partition by the space you want to give Ubuntu (50 GB min) to create unallocated space.
3. [Disable Fast Boot](https://www.lifewire.com/disable-fast-startup-in-windows-10-5094422):
   - May require [enabling/disabling hibernate](https://docs.microsoft.com/en-us/troubleshoot/windows-client/deployment/disable-and-re-enable-hibernation) first.
4. [Disable Secure Boot](https://docs.microsoft.com/en-us/windows-hardware/manufacture/desktop/disabling-secure-boot?view=windows-11).

Then follow [this dual-boot tutorial](https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/), grabbing the [24.04 image](https://releases.ubuntu.com/noble/) instead and [Rufus](https://rufus.ie/en/) for the flash drive. We also have pre-flashed Ubuntu 24 USB drives on the team, if you're around in person you can just ask for one instead.

After install, update your drivers:

- Software Updater → Settings → Additional Drivers.
- Apply a proprietary tested driver if one's listed.
- If audio doesn't work, run `sudo alsa force-reload`.

## Ensure Your Time and Date is Correct

Windows and Ubuntu handle the hardware clock differently, which breaks APT if left unfixed. Set the time manually (search "time" in the Windows settings). See [this fix](https://askubuntu.com/a/169384) if you're tired of switching it back and forth.

:::tip[Continue with:]
[Native ROS Installation](/setup/native-install).
:::
