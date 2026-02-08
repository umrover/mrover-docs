---
title: "Teleop Quickstart"
---
## Additional resources: [Vue](/teleop/vue-introduction), [Bootstrap](/teleop/bootstrap-introduction)

Once you have completed the installation from the [Install ROS](/getting-started/install-ros) page, checkout the teleop branch by doing

```bash
git checkout teleop
```

This is the main teleop branch. Features will be developed checked out from and merged into this branch. In the mrover home directory, run the following commands:

```bash
./build.sh
./ansible.sh build.yml
./ansible.sh teleop.yml
```
then
```
./build.sh
```

This will install bun, our javascript runtime, and other packages required for the application to run. ```./build.sh``` recompiles the backend code. 

For any errors you might encounter, check out [Teleop FAQ](/teleop/faq) for fixes and questions. 

Make sure to set up your editor. We recommend VSCode, as does most of the EECS classes you will take. Be sure to install the Python, Vue, Bootstrap Intellisense, and Prettier extensions, then enable Auto Save. 

Prettier will help you format your code every time you save (Ctrl-S). 

Bootstrap Intellisense will help you autocomplete css classes for faster development. Hover over a class, and it will show you the underlying css for that class. 

Now, you should head to and complete the [teleop starter project](/teleop/starter-project). 

## Launch Commands

```bash
ros2 launch mrover basestation.launch.py
```

This command starts both our **frontend and backend** services. You should be able to see the application running on [localhost:8080](http://localhost:8080). Hit ```ctrl-c``` on your terminal to kill the application. 

---

```bash
ros2 launch mrover simulator.launch.py
```

This command launches the simulator, which gets rover coordinates published to the frontend. This is useful when testing and interacting with things like the map. 

These aren't the only launch commands, but these two are the ones that we will use most frequently. For more, check out the ```/launch``` directory. 

If you encounter any problems, make sure to check out the [Teleop FAQ](/teleop/faq) and see if an answer there solves your problem. 

## Debugging

Unlike your course project, print statements aren't exactly going to work here. You can of course use ```console.log()``` when working with the frontend, but to verify the data being sent through the backend, here's how to debug topics and services. 

### Debugging Topics: 

```
ros2 topic list
```
Lists all the active topics that are being published to or subscribed to. You can use this to ensure that your topic exists and is correct. 

```
ros2 topic echo /<topic name>
```

This command listens to the topic specified by ```<topic name>``` and prints to the terminal the data being published. For example:

```
$ ros2 topic list
/greetings
/parameter_events
/rosout

$ ros2 topic echo /greetings 
data: Hello there, let's debug topics!
---
data: Hello there, let's debug topics!
---
```

---

### Debugging Services: debug_service.py
Debugging a service call is not as straightforward. For this, we have provided a file ```/scripts/debug_service.py```

Open it in your editor, replace ```SERVICE_TYPE``` and ```SERVICE_NAME``` for the service you are debugging, and run ```python3 scripts/debug_service.py``` to start the service. 

---

# Extras

For an introduction on Vue, click [here](/teleop/vue-introduction)

For an introduction on Bootstrap, click [here](/teleop/bootstrap-introduction)

For more info on ROS commands, check out [this](https://roboticsbackend.com/ros2-topic-cmd-line-tool-debug-ros2-topics-from-the-terminal)


