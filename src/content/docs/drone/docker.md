---
title: "Docker Setup (Experimental)"
---

Docker enables running a different operating system environment in a container. In our case this allows you to run Ubuntu 22.04 with necessary ROS2 and PX4 dependencies even if your operating system does not support it.

Before beginning these steps, install QGroundControl locally on your system: https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html 
## Windows (WSL)
1. In PowerShell, install WSL:
```
wsl.exe --install Ubuntu-22.04
```
2. Open WSL (Ubuntu 22.04 shortcut) and install git:
```
sudo apt install -y git
```
3. Set up GitHub SSH key.
4. Install Docker Desktop in Windows: https://docs.docker.com/desktop/setup/install/windows-install/

5. Clone mrover-drone repo in WSL:
```
git clone git@github.com:umrover/mrover-drone.git
```
6. Set correct versions for dependencies:
```
cd mrover-drone/deps
cd px4_msgs
git checkout release/1.15
cd ../px4-ros2-interface-lib
git checkout 1.4.0
```
7. Build the docker container:
```
eval $(ssh-agent -s)
docker compose -f docker/compose.yml up -d --build mrover-drone
```
After the build process finishes, check the setup logs by running:
```
docker compose -f docker/compose.yml logs --follow  mrover-drone
```
Look for the following output to ensure the container is set up correctly:
```
mrover-drone  | + exec /bin/zsh
mrover-drone  | (anon):12: character not in range
```
8. In future runs, start the docker container by opening the docker desktop app and running `./docker_up.sh` in WSL.
9. To enter the docker container, run:
``` 
docker compose -f docker/compose.yml exec -it mrover-drone /bin/zsh
```

## MacOS
Doesn't really work. Gazebo did not run with OpenGL. For X11 apps use XQuartz and enable network connections from remote clients. 