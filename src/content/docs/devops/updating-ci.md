---
title: "Updating CI"
---
When a PR introduces a new dependency it is sometimes necessary to update the CI job if that dependency cannot automatically be pulled in with rosdep. The CI job runs on a Docker container pulled from Dockerhub, the following explains how to bump the Docker container to make your PR build.

Docker containers on Dockerhub are built from a Dockerfile, which is contained in the root of this repository. This process of building and pushing changes is not automatic. The Dockerfile must be built into a container on your system locally, then you can push the updated binary container to Dockerhub. 

1. Install Docker Engine on your system: https://docs.docker.com/engine/install/ubuntu/
2. Make your desired changes to the Dockerfile (the syntax is fairly intuitive) 
3. `docker build . -t umrover1/ros:latest`. This may take a while.
4. You should see your build container with with `docker image list`
5. Now do `docker run -it umrover1/ros:latest` to get an interactive terminal in a new session of your container
6. Create a new catkin workspace and git clone the repository `source /opt/ros/noetic/setup.zsh` and run `catkin init`
7. Run `catkin build` and confirm that your change allows things to build as expected
8. Note that if the build fails, you can use the interactive session to prototype changes such as with `apt install`. However, any changes made here will not be reflected in the Dockerfile, so you MUST make those changes in the Dockerfile and restart the processes from step 3 once you've done so. 
9. `exit` from the container 
10. Perform `docker login`, dm Ashwin for the mrover Dockerhub credentials 
11. `docker push umrover1/ros:latest`



**Warning: It is absolutely imperative that the Dockerfile in the repo remains synchronized with the Docker container on Dockerhub. As soon as you push an update to the container do one of the following:**
1. If you have permissions, push your change to the Dockerfile directly to master, bypassing the PR review processes
2. Otherwise, open a PR and ping Ashwin in #dev-ops so he can merge it for you