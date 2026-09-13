---
title: "Portable ROS Installation (Unofficial)"
sidebar:
  label: "Portable ROS Installation (Unofficial)"
---

:::caution
This path is **unofficial** and maintained by software leadership. Only native Ubuntu 24.04 is officially supported.
:::

:::danger
ESW is **NOT** supported by this installation.

Perception is **SEMI** supported by this installation. Ask your lead.
:::

## Supported Platforms

The portable environment is built on [pixi](https://pixi.sh) and currently supports:

- macOS on Apple Silicon (arm64)
- macOS on Intel (x86_64)
- Linux x86_64, on a non-Ubuntu 24 distro

Anything else, including Windows and Linux on arm64, is not supported by the portable path.

For Windows, please install Ubuntu 24.

## Install

Run:

```
curl -fsSL https://setup.mrover.org/bootstrap-portable.sh | bash
```

This clones the repo to `~/mrover-ros2`, installs Homebrew on macOS if you don't have it, installs git/git-lfs/Ansible with your system's package manager, then runs Ansible and sets up your environment using Pixi.

If you already have the repo cloned, skip `bootstrap-portable.sh` and run `./setup-portable.sh` directly.

You'll also need an SSH key set up with Github to clone. See [Github's guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) if you haven't done that yet.

Once it finishes, log out and back in, open a new terminal, and run:

```
mrover
./build.sh
```

`mrover` jumps you into the repo and activates your environment, except here that environment is the pixi shell, not your system Python/ROS install.

## Sanity Check

Open a new terminal, run `mrover`, and run:

```
ros2 launch mrover simulator.launch.py
```

Confirm the simulator launches.

## Keeping Dependencies Up to Date

Run the portable playbook:

```
./ansible.sh dev-portable.yml
```
