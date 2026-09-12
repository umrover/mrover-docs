---
title: "URC vs. CIRC Switch"
---
**Context**: The [URC](https://urc.marssociety.org/home/requirements-guidelines) and [CIRC](https://circ.cstag.ca/2023/tasks/#traversal) competitions have different rules and missions.

**Problem**: Currently, the master branch of our code is focused on following the URC rules. Some CIRC changes were implemented on another branch, but this becomes hard to keep track of. If there are changes in one branch that are necessary for both, merging one into the other may cause a lot of conflicts, or we would have to edit the two branches separately for the same change. 

**Solution**: Create a “switch” for URC mode vs. CIRC mode on the master branch and add necessary changes to the code for the CIRC competition. Handle the switch off between the two within the code.

**Interface** (more to be added)

**Rough Steps:**
1. Read through this year’s [URC rules](https://urc.marssociety.org/home/requirements-guidelines) and previous years' CIRC rules ([2023](https://circ.cstag.ca/2023/tasks/#traversal), [2022](https://circ.cstag.ca/2022/tasks/#reactor-patrol-route-traversal-task)) to find differences **DONE**
2. Remove all code from the master branch related to gate since it is no longer needed by either competition **DONE**
3. Add a value in a .yaml file that we will set for either URC mode or CIRC mode **DONE**
4. Make CIRC changes. Throughout the code there will be if statements to perform certain actions based on the mode we are in 


CIRC changes (more can be added):
1. Create a new publisher, mTagIdPub, to publish the AR tag ID number (in perception code)
2. Consider all tags when searching for a waypoint (only first two tags must be visited in order) 
3. Convert heading into GPS location for extra waypoint 
>“One location will be a certain distance and compass heading from the second location. The distance and heading will be clearly marked on a sign at the second location in a human readable format. The compass heading will be displayed in magnetic heading. The distance will be displayed in meters.”
4. Look for a light beacon and drive towards it for one of the waypoints. 
>“A location will have a light beacon. The rover must scan the terrain for the light and navigate towards it. The light will be at least 50 cm above the ground. The light will have a consistent strobe pattern. The light will be amber in colour. The light will emit in 360°. A sample video of the light beacon strobe pattern in sunlight is available”.
