---
title: "Path Planning"
---
# Context: 
Last year, we had a lot of development in our path planning and path execution. In particular, we added path smoothing after A* generation and pure pursuit as a new drive controller. There was also development towards inward spiraling on the rover. 

# Problem:
During the auton mission, we are limited on time. As such, we need to drive in a way that will save us the most time possible. This includes doing research into different ways to improve the rovers speed and ability to detect objects.

# Solutions:
## [Hybrid A*](/autonomy/navigation/hybrid-astar)
A*'s path is often quite jagged, and creates a path in any direction regardless of the rovers heading. To solve this, last year we implemented path smoothing; however, we never solved the issue of the path being created in any direction. Our rover can skid steer, which means this is not too much of a problem, but there are considerations on if prioritizing directions that the rover drives in will increase overall speed.

## Global A*
With perceptions integration with LIDAR data, we are now able to have a long-term A* path. Currently, we compare a straight line path with a local A* calculation. Instead, we are now able to attempt to compare a local A* calculation with a long-term A* path.

## [Search Trajectory Research](/autonomy/navigation/search-trajectory)
As detailed in the [navigation wiki](/autonomy/navigation/overview), the high-level decision-making for the rover's navigation is based off of a state machine. In the search state, we currently spiral from a center waypoint outwards within some radius. However, URC rules state that we don't have to drive directly towards objects. In particular, as long as we detect an object, we are allowed to stop and move onto the next waypoint. This brings into discussion of potential search trajectories which prioritize convering the search area in a matter besides full land coverage.

## [Modulation](/autonomy/navigation/modulation)
To enable the rover to use Hybrid A* and switch between different search trajectories, it is necessary that we have an option for operators to choose between these options. Having a more modular system will allow an operator to have the option to switch and choose different path planning algorithms and search trajectories. 
