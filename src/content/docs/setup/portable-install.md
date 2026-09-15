---
title: "Portable ROS Installation (Unofficial)"
sidebar:
  label: "Portable ROS Installation (Unofficial)"
---

:::caution
This path is **unofficial** and maintained by software leadership. Only native Ubuntu 24.04 is officially supported.
:::

## Supported Platforms

The portable environment is built on [pixi](https://pixi.sh) and currently supports:

| Subteam      | Non-Ubuntu24 Linux | Mac                |
| ------------ | ------------------ | ------------------ |
| Navigation   | Supported          | Supported          |
| Perception   | Semi-supported*    | Semi-supported*    |
| Localization | Supported          | Supported          |
| ESW          | Supported          | Mostly-supported** |
| Teleop       | Supported          | Supported          |
| Drone        | Supported          | Supported          |

*Perception members should consult the perception lead directly

** ESW mainly uses the `mrover-esw` repo. The `mrover-ros2` repo only contain the hardware bridges. It is acceptable for an ESW member to be unable to build ESW on the`mrover-ros2` repo

:::danger
Anything else, including Windows and Linux on arm64 (snapdragon), is **NOT** supported by the portable path.
:::

For Windows, please install Ubuntu 24 directly. WSL2 will **NOT** work.

For members with Snapdragon laptops, we recommend getting a loaner from the University [here](https://its.umich.edu/computing/computers-software/sites-at-home).

## Set Up Your SSH Key

To clone our code repository from GitHub, you need to add SSH keys so that Github can authenticate you.

Github explains the SSH process [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).

TL;DR:

- complete the **"Generating a new SSH key"** section detailed [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
- run `cat ~/.ssh/id_ed25519.pub`, copy output
- visit [here](https://github.com/settings/keys)
- click `New SSH key`
- paste output in

## Install

Run:

```bash
curl -fsSL https://raw.githubusercontent.com/umrover/mrover-ros2/skj/portable/bootstrap-portable.sh | bash
```

This clones the repo to `~/ros2_ws/src/mrover-ros2`, installs relevant packages and libraries, then runs Ansible and sets up your environment using Pixi.

If you already have the repo cloned, skip `bootstrap-portable.sh` and run `./setup-portable.sh` directly.

You'll also need an SSH key set up with Github to clone. See [Github's guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) if you haven't done that yet.

Once it finishes, log out and back in (or reboot), open a new terminal, and run:

```bash
mrover
./build.sh
```

`mrover` jumps you into the repo and activates your environment, except here that environment is the pixi shell, not your system Python/ROS install.

## Sanity Check

Open a new terminal, run `mrover`, and run:

```bash
ros2 launch mrover simulator.launch.py
```

If the simulator launches, you have successfully set up the codebase and ROS environment.

## Keeping Dependencies Up to Date

Run the dev-portable playbook:

```bash
./ansible.sh dev-portable.yml
```
