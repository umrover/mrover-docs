---
title: "Modulation"
---

# Concept  

The current implementation of navigation hard codes which algorithms are used for path planning and search patterns. Because of the proposed projects this year, we need to allow operators to determine the path planning algorithm and search pattern. This will make the rover's path planning modular, allowing for AB testing on the rover. By using A* as a baseline, we can test algorithms efficiency.

## Integration

Navigation will create two topics that Teleop will subscribe to: '/path_algorithm' and '/search_pattern'

## Process

* Change the current way A* is called so that we are able to switch between different path algorithms.
* Change the way that the search trajectory is generated so that we are able to switch between search patterns.
