---
title: "Install ROS"
---
## Operating Systems and Requirements

We highly recommend running Ubuntu 22.04 LTS natively. This will be by far the smoothest experience. There are many tutorials on dual booting out there if you are not comfortable with daily driving Linux. You will likely need 50 GB of space for your Linux partition. Perception members should aim for 100 GB due to the large nature of NVIDIA packages. 

_**Please note that we will only officially support Ubuntu 22.04 LTS running natively**_. All other options are not as thoroughly tested and we cannot provide the same level of support if you run into any issue. However, here are some other options you can try to explore: 
1. macOS see [here](/getting-started/install-ros-macos)
2. Ubuntu 22.04 LTS in a VM (Makes things like USB and GPU access difficult).
3. Other distro of choice by compiling from source (i.e. Arch and AUR) or using RoboStack for premade packages

Some other things to know:
* WSL on Windows 10 is missing a lot of features and has proven to be much more trouble than its worth. WSL on Win11 still leaves a lot to be desired and does have strange issues from time to time, but seems to be much better. WSL is by far the worst option listed though in any case. 

### Dual-Booting Ubuntu 22.04 LTS from Windows
The following steps may or not be necessary prerequisites. If you find other important steps, have better tutorial links, or don't think something is necessary for certain Windows devices, please add your knowledge.
1. Check if you have BitLocker by right clicking on the Windows icon on your taskbar and clicking on Disk Management. In the Disk Manager, check if any of your partitions (usually the largest one) say "BitLocker encrypted" on them. If so, you have BitLocker and should try to [suspend it](https://4sysops.com/archives/disable-bitlocker-on-windows-11/#rtoc-3). If there is no option to suspend it, just make sure you can [find your backup keys](https://support.microsoft.com/en-us/windows/finding-your-bitlocker-recovery-key-in-windows-6b71ad27-0b89-ea08-f143-056f5ab347d6) either on your Microsoft account or on a flash drive and you will be fine.
2. [Disable Fast Boot](https://www.lifewire.com/disable-fast-startup-in-windows-10-5094422) (You may need to [enable or disable hibernate](https://docs.microsoft.com/en-us/troubleshoot/windows-client/deployment/disable-and-re-enable-hibernation))
3. [Disable Secure Boot](https://docs.microsoft.com/en-us/windows-hardware/manufacture/desktop/disabling-secure-boot?view=windows-11)

For a tutorial on dual booting Ubuntu, read [this page](https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/). Below is a summarized version of the tutorial:

Next, you should right click on your windows icon on your taskbar and then click on Disk Management. once the disk management window opens, you should look for the largest partition on your drive. Its size should also match up with the size of your C drive if you open File Explorer and go to My PC. Once you have identified this drive, right click on it and select Shrink Partition. This will open another window, that will allow you to select an amount to shrink the drive by. This will be the amount of space your Ubuntu install will have, so we recommend at least 50-60GB if possible. Enter your amount (in MB, so multiply GB by 1000) and then press Shrink Partition. Once this is complete, you should see a new partition at the end of your largest partition that is gray and says free space or unallocated.

Now you are ready to boot into your Ubuntu USB. Plug in your flash drive, then search "uefi" in the windows search bar. You should see an option pop up called "Change advanced startup options", click on this and then select Restart now. Once you see a blue screen labeled "Choose an option", select "Use a device" and then select the USB device corresponding to your flash drive (usually this will say something like UEFI and/or the brand name of your flashdrive, if it's not obvious which one to choose, just repeat this process and try them all). Your system should then reboot and you should see a black menu with a white selector bar, with various options relating to Ubuntu. Press enter to select the first option.

The Ubuntu installer will then boot up on your computer. When asked whether you'd like a trial or full version of Ubuntu, select the full version, then follow the prompts. Choose normal (not minimal) installation, do not connect to wifi, and do not select to install 3rd party software. Once you reach the page labeled "Installation Type" choose the "Something else" option. Then select the partition labeled "free space" that corresponds to about the same size that you freed up in the Windows Disk Manager. Right click on this partition and choose add (or press the plus button in the bottom left). Change the mount point to "/" and then choose Ok. Finally, click install now. It will warn you that some partition is going to be modified, double check that it's the correct one that you freed up in Windows. If you have any confusion about this part, read the tutorial linked above or feel free to ask a lead or other member who has done this before. Once this is done, follow the rest of the prompts until Ubuntu is installed.

Now that Ubuntu is installed, to switch between Ubuntu and Windows all you have to do is restart your computer. When it boots up, it should show the same black menu with the white selector bar. Use the arrow keys to select either Ubuntu or Windows Boot Manager, depending on which operating system you want to use.

If you need to make your own bootable flash drive, follow [this link](https://releases.ubuntu.com/jammy/) to download the desktop image of Ubuntu 22.04, and download [Rufus](https://rufus.ie/en/).

After everything is set up and you're getting familiar with Ubuntu, you can update drivers like so:
1. Search (by pressing the windows key) for "update manager" and select "software updater".
2. When a box says your computer is up to date, click settings.
3. Open the Additional Drivers tab.
4. Look for a proprietary, tested driver. If one exists, select it and click "Apply changes".

If you can't get audio to work, open the terminal and enter `sudo alsa force-reload`.

Again, please update this section if you come across anything else that needs to be done for dual booting from Windows.

## Ensure Your Time and Date is Correct

Windows and Ubuntu use your hardware clock different by default. Please set the time correctly by hitting the windows key (or command key on macOS), searching for time, clicking on the first settings link, and configure it manually. If you do not do this the APT package manager will fail to work. If you are annoyed by constantly switching this see: https://askubuntu.com/a/169384

## Set Up Your SSH Key

To clone our code repository from Github, you need to add SSH keys so that Github can authenticate you. If you just installed Ubuntu, this will not be configured automatically.

Github explains the SSH process [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account). Make sure to complete the linked prerequisites to generate an SSH key on your machine. Then add the key to your Github account following the instructions on the page. If you're interested, you can also check out commit signing (linked near the top of that page).

## Install ROS Natively

This is the recommended way to install ROS, and the only one we will guarantee full support for.

#### Install

First, run:
```
sudo apt update && sudo apt upgrade
```

Then run `wget -O bootstrap.sh https://raw.githubusercontent.com/umrover/mrover-ros2/main/bootstrap.sh && chmod +x ./bootstrap.sh && ./bootstrap.sh`

Once the script completes, **restart your machine**. Then open a terminal and run:
```
mrover
./build.sh
```

## Update Dependencies
The dependencies to our code often change, which can prevent you from running your code. You usually encounter this when your catkin build fails, saying a dependency is missing. When you encounter this problem or just want to make sure all your dependencies are up to date, you can run `./ansible.sh build.yml`

## Setup MRover codebase 

To run a quick sanity test, **open a new terminal window** and try `ros2 launch mrover simulator.launch.py`. RViz and the sim should pop up and the only red errors present should say "Frame [map] does not exist".

If the sensitivity of the simulator seems too high (and you are running ubuntu 22), edit `/etc/gdm3/custom.conf` using the command `sudo vim /etc/gdm3/custom.conf`. Then change line 7 from `#WaylandEnable=false` to `WaylandEnable=false` and then **restart your machine**. This line can be changed by pressing `:+7`, `x`, `:+w+q+enter`. Ask one of the leads if you need help using `vim`.