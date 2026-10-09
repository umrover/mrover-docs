---
title: "Hybrid A*"
---

# Concept  

Hybrid A* is a path planning algorithm which finds the optimal path from A to B. It is a modification of the algorithm A* that also uses the rover's heading. Instead of considering 8 grid directions around the rover, instead one is able to define points in any direction. Additionally, one is able to apply cost to these directions so some are prioritized over others. An introduction to Hybrid A* can be found [here](https://medium.com/@junbs95/gentle-introduction-to-hybrid-a-star-9ce93c0d7869). An overview of our A* can also be found [here]().

<img width="706" height="363" alt="Screenshot from 2026-09-12 19-44-42" src="https://github.com/user-attachments/assets/0f3a2e65-1f1d-476c-82b1-a299bb55d4b9" />

## Process

At each step, A* evaluates a current node by combining two values: g(n) and h(n).

- `g(n)` - the actual cost to reach the current node `n` from the starting point
- `h(n)` - a heuristic estimate of the cost from the current node `n` to the ending point

Each node is then given a value based on the following algorithm: `f(n) = g(n) + h(n)`. When making a path A* considers 8 different directions which can be represented by the 8 directions on a compass. Instead of considering these directions, do research into points around the rover that it can follow.

In addition to adding the cost of going to that point with the current high cost in that area, we will also add a cost depending on how it will affect the rovers heading. 

For example: Cost = C_distance + W_s(C_steering) + W_o(C_obstacle)
