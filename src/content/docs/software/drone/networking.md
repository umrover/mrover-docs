---
title: "Multi-Machine ROS2 Networking"
---

## Configure LAN IPs

In order for machines to communicate, they must be on the same subnet. After connecting the computer to the network over ethernet cable, run 
```
ip a
```
to view a list of network interfaces. Look for one that starts with `eth` or `en`. You can view the local ip that was assigned, it will likely be 192.168.1.X\24

24 is the network mask.

To set your local ip, click on 'wired connected'->'wired settings'->'settings gear'->'ipv4'

Set the IPv4 Method to "Manual" and set the IP to 192.168.1.X. Do this for all computers. (X is usually some number 1-100, but definitely ess than 256)

Set the netmask to 24 or 255.255.255.0 (same thing)

Gateway doesn't matter.

## Configure ROS_DOMAIN_ID
In order to prevent accidental cross talk between machines running ROS2 on the same network, both machines must set the same ROS_DOMAIN_ID environmental variable.

[Link to ROS2 Docs](https://docs.ros.org/en/eloquent/Tutorials/Configuring-ROS2-Environment.html)

1. in ~/.zshrc, add the line:
```
export ROS_DOMAIN_ID=X
```
Where X is some agreed upon value.

2. Close and reopen all terminals running ROS

3. Now any topics on either computer should appear on the other