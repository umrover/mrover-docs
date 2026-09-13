---
title: "Setting up the Jetson"
---
:::caution
The recovery-mode boot sequence below is unverified. Someone on ESW with access to a Jetson needs to confirm it still matches; treat that part as unreliable until then.

For everyday dev setup (not initial flashing), just run `./setup.sh` from the repo on the Jetson itself; it detects the Jetson automatically.
:::

## Flashing

NVIDIA SDKManager is no longer used. Flashing is fully automated through Ansible: run `./ansible.sh jetson_flash.yml` from the repo on the host computer connected to the Jetson (not on the Jetson itself). See `ansible/jetson_flash.yml` and `ansible/roles/jetson_flash` in `mrover-ros2` for the exact steps; as of now it targets a Jetson AGX Orin devkit on L4T r39.2.1 (Ubuntu 24.04 "noble"), downloading the driver package, root filesystem, and cross-compiler directly from NVIDIA, then building a custom kernel and the PCAN drivers before flashing.

Make sure `tlp` is not installed on your host system!!! For some reason it breaks the flashing.

## Boot the Jetson in recovery mode

TODO
