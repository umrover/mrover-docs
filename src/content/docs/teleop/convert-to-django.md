---
title: "Convert to Django"
---
**Context:** Very similar to the [starter project](/teleop/starter-project), we are implementing a backend so no crucial data is lost when reloading the webpage. We also need to make sure our current GUIs are up-to-date with the current [rules](https://urc.marssociety.org/home/requirements-guidelines). Quite a few rules have changed so each component may or may not be affected by it.

**Problem:** All of our components are written only in the frontend Vue. We need to implement Django as well to have the data persist.

**Solution:**
Convert every component to implement Django with it.

**Interface:**
Vue <=> Django via Websockets. Django <=> ROS via topics. See the starter project for more details.

**Rough Steps:**
* Take the Vue component and add it to the Django directory
* Remove/change anything that isn't in the rules this year
* Update component to Vue3 
* Figure out what needs to be sent to ROS or be saved in a database and put that in Django.