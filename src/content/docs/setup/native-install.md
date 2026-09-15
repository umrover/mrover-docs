---
title: "Native ROS Installation"
sidebar:
  label: "Native ROS Installation"
---

This page installs the MRover codebase and its dependencies for Ubuntu 24.

:::caution
If you don't have Ubuntu 24.04 running natively yet, do that first: [Installing Ubuntu](/setup/installing-ubuntu).

If you are on a Mac or want to use another Linux distro, see [Portable Installation](/setup/portable-install).
:::

## Set Up Your SSH Key

To clone our code repository from Github, you need to add SSH keys so that Github can authenticate you. If you just installed Ubuntu, this will not be configured automatically.

Github explains the SSH process [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account). Make sure to complete the linked prerequisites to generate an SSH key on your machine. Then add the key to your Github account following the instructions on the page. If you're interested, you can also check out commit signing (linked near the top of that page).

TL;DR:

- complete the **"Generating a new SSH key"** section detailed [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)
- run `cat ~/.ssh/id_ed25519.pub`, copy output
- visit [here](https://github.com/settings/keys)
- click `New SSH key`
- paste output in

## Install

Run:

```bash
curl -fsSL https://setup.mrover.org/bootstrap.sh | bash
```

This clones the repo to `~/ros2_ws/src/mrover-ros2` and runs Ansible to install ROS, our toolchain, and your dev environment (zsh, oh-my-zsh, VSCode, etc).

If you already have the repo cloned, skip `bootstrap.sh` and run `./setup.sh` directly.

Once it finishes, **log out and back in** (or restart) so your shell picks up the changes, then open a new terminal and run:

```bash
mrover
./build.sh
```

`mrover` jumps you into the repo and activates the build overlay. You'll want to run it in every new terminal you use for MRover work. There's no separate workspace to `cd` into anymore: the repo you cloned _is_ the workspace.

## Sanity Check

Open a new terminal, run `mrover`, and try:

```
ros2 launch mrover simulator.launch.py
```

RViz and the simulator should pop up. The only red errors present should say "Frame [map] does not exist".

If the simulator's mouse sensitivity seems too high, edit `/etc/gdm3/custom.conf` using `sudo vim /etc/gdm3/custom.conf`, change line 7 from `#WaylandEnable=false` to `WaylandEnable=false`, then **restart your machine**. Ask a lead if you need help using `vim`.

## Keeping Dependencies Up to Date

Our dependencies change over time. If a build starts failing complaining about a missing dependency, or you just want to make sure you're current, run:

```
./ansible.sh dev.yml
```

from inside the repo. This is exactly what `setup.sh` ran for you the first time.
