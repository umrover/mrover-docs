---
title: "Autonomy Overview"
---
# Autonomy
The following sections will introduce you to the auton mission at URC, an overview of MRover's autonomy system architecture, and some general expectations for working on autonomy on MRover. 

## The Autonomy Mission 
The ultimate authority on the autonomy mission is the official URC rules. We highly recommend you read at least section 1.e "Autonomous Navigation Mission" [here](https://drive.google.com/file/d/1LY5VX9Psww8Bx4ub_AWMgUlajPZQ2X2A/view). What follows is a brief summary of the autonomy mission to recap the most important points. 

Autonomy is all about navigating through the desert with no driver input. The rover must traverse a fairly rough course through a series of waypoints. It must make all navigation decisions based on its software and sensor input like GPS, gyroscopes, and computer vision. There are four kinds of waypoints on the autonomy course:
1) Post (GPS): A GPS Waypoint is provided that the rover must drive to within 3m. These are the simplest type of waypoint, but they do require high global localization accuracy to reach. There are three of these in the course.
2) Post (Fiducial): This waypoint has a single [ARUCO](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) fiducial marker. We are given a GPS coordinate that could be up to 20m away from the post, and from there we must search for the fiducial marker using computer vision and bring the rover to within 2m of the marker. There are also three of these in the course.
3) Gate: This is essentially two posts separated by a short distance. We must drive the rover through this gate using computer vision. This is by far the most demanding part of the course because it requires stable vision, high relative localization accuracy, and well tuned driving. There is one gate at the end of the course. We did not successfully complete the gate at URC in 2022. 
4) Post (Mallet/Water Bottle): During the auton mission there is one waypoint which has a single [Mallet](https://www.amazon.com/Neiko-02848A-Unibody-Checkered-Resistant/dp/B000REO0TQ/ref=sr_1_11_sspa?crid=E3Y36R4C1L1I&keywords=rubber%2Bmallet&qid=1693373825&sprefix=rubber%2Bma%2Caps%2C179&sr=8-11-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9tdGY&th=1) object and another GPS waypoint which has a single [1L Wide Mouth Water Bottle](https://www.amazon.com/Nalgene-Mouth-Water-Bottle-1-Quart/dp/B000KA6AOU?source=ps-sl-shoppingads-lpcontext&ref_=fplfs&psc=1&smid=ATVPDKIKX0DER). We are given a GPS coordinate that could be up to 10m away from the object, and from there we must search for the object using a machine learning based object detector to bring the rover to within 2m of the respective object.

To watch our performance during the autonomous mission last year click the video below: 
[![video](https://i9.ytimg.com/vi_webp/IvkKQNy5MkM/mq2.webp?sqp=COjTwLcG-oaymwEmCMACELQB8quKqQMa8AEB-AH-CIAC0AWKAgwIABABGGUgVihNMA8=&rs=AOn4CLCryWfeQSoD_usxzRlwakKAX488ug)](https://www.youtube.com/watch?v=rYeIg2el6fw&feature=youtu.be)


## High Level System Architecture 
* The autonomy system is made up of three main components: perception, localization, and navigation. The perception and localization microservices take in data provided by the sensors and process it into easily interpret-able data which navigation can use in its decision process. Using the information provided by perception and localization navigation then controls the rover's movement using a [FSM](https://en.wikipedia.org/wiki/Finite-state_machine). It is the FSM which combines all of the data to issue drive commands to control the rover's movement.
![auton diagram drawio](https://user-images.githubusercontent.com/10037572/188723988-dece8e97-9eb3-49e7-90bf-a35d2396850a.png)


## TF Tree 
The following TF tree is representative of the transforms that the rover is aware of during routine auton operation. AR tag readings will not always be present if there are no recent detections. This is a snapshot of `ros2 run rqt_tf_tree rqt_tf_tree`

![tf](https://user-images.githubusercontent.com/10037572/188722404-8eb8f095-acdb-4fac-9cd8-8a6836004266.png)


## Process Graph 
The graph below shows all of the nodes that will run on the rover during the autonomy mission, as well as what they communicate to each other. This is similar to the high level system architecture but slightly more granular as it is broken down into individual software processes as opposed to subsystem components. This is a snapshot of `ros2 run rqt_graph rqt_graph` from simulation.

![rqt_graph](https://user-images.githubusercontent.com/10037572/188722418-c10a3ba4-5d1f-4f28-9b9e-7d9a628e0399.png)


## Guidelines for Autonomy Members
These guidelines are mostly aimed at creating a positive work culture for auton members. Additionally, auton has dealt with our fair share of [technical debt](https://en.wikipedia.org/wiki/Technical_debt) in the past. Some of the guidelines here are designed to counteract that in the future as we rebuild our codebase in ROS.  

1. **Be kind and have a good time**. We want Mrover to be an enjoyable, positive experience for everyone. While we obviously take competing seriously, we don't want to take it _so_ seriously that people feel stressed out. 

2. **Strive for a blameless culture**. When something isn't working, its on everyone to get that thing working, not just the person who wrote the code. We never want to point fingers. Having a blameless culture means accepting accountability for yourself (never pushing it onto others) and being focused purely on solving problems instead of assigning blame.

3. **Be humble.** Try to favor existing open source software packages and libraries as opposed to implementing something from scratch yourself. Particularly for something very common: don't reinvent the wheel. It is not worth it to write your own matrix library or Kalman Filter. Someone who is an expert in a particular domain has likely written more robust, better documented, and faster implementation than what we will be able to hack together in 6 months before SAR. Using off the shelf code is critical to reducing technical debt for future MRover members. Even though you it can be cool to write your own matrix library from scratch, this creates issues later on. Save stuff like that for off-season or personal projects.

Of course, there are times when it makes sense to implement something yourself. For example:
* There are no existing implementations
* The existing implementation is overly complex and you have a relatively simple use case (e.g. our own SE3 class vs Sophus) 
* You can make a meaningful improvement in speed or functionality by implementing it yourself. If this is the case, you should prove it with our design review process 

4. **Be egoless.** It is critical to hear everyone's opinions and make compromises. By taking into account multiple ideas, we have a better chance of arriving at the best solutions. This means that everyone will not their own way every single time. Part of being an engineer on a large team is putting good engineering practice ahead of personal incentives. The best engineers are egoless - focus on finding the best solution together.  

5. **Favor readability and ease-of-use** where possible. Historically, MRover code has never been limited by runtime performance. If you find yourself over-optimizing (like writing a [GPU accelerated point cloud processor](https://github.com/umrover/perception-gpu-obs-detector)) at the cost of usability (needing a CUDA-capable GPU to run the code), you are likely making a mistake. 

6. **Document.** Every year we will onboard dozens of new people to software. If your code does not get documented, getting new people familiar with our software becomes much more difficult. Undocumented code is not fun to work with for anyone except the author. In the long term, the knowledge of how it works will get lost once you graduate. That code then becomes technical debt and there's a good chance it will just need to get re-written. 

7. **Do high quality work**. We are always in a hurry. Being in a hurry does not justify writing sloppy code or continuing to use existing infrastructure that has proven to be chronically broken. If something is not right, its on everyone to identify that and fix it. When you write something new, optimize quality from the start by taking full advantage of extensive peer review in the PR stage. 

Some other notes: 
* Put effort into reviews. It can be tempting to just hit approve and move on. We've all been there. Nobody wants to read pages and pages of code on a long PR. However, we want all code on the rover to be of the highest quality possible. If you see something that needs to be re-written please do not be afraid to say so. Give constructive criticism and get the implementation right during the PR stage, so that it does not become technical debt. It's always better to be safe than sorry, and we encourage you to flag potential issues even if you are unsure about them.
* Testing must be rigorous. "It looked right" is not rigorous. If you are trying to convince your subteam that the IMU is working, you need to be able to be as concrete as "the yaw reading deviates from the ground truth by a maximum of 1 degree when spun 50 degrees on a turntable 10 different times".  
* Put information about testing and validation in PRs. This could be simulator or unit tests, or real rover tests. This is really helpful for reviewers and record keeping. 