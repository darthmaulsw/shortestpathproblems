# Pathfinding Algorithms in C++

This repository explores and implements several fundamental graph-search algorithms in C++, including Greedy Best-First Search, Uniform Cost Search (UCS), and A* Search, and compares their performance on the real-world problem of finding the shortest path between two geographic points in Indiana.

## Problem Overview

Finding the shortest path between two points on a map is one of the central problems in graph theory and pathfinding. In this project, Indiana's streets are represented as a graph where each road segment has a start point, an end point (given in latitude and longitude), and an associated distance. Once this data is represented as a graph, classical search algorithms can be used to determine how to efficiently travel from one location to another.

Many graph-related problems can be represented using grids or geometric data, and a common approach is to convert such representations into adjacency lists or adjacency matrices. However, doing so often requires storing coordinate pairs and repeatedly packing/unpacking these into queues or other structures. An alternative approach, especially in multi-dimensional grids, is to maintain a queue per dimension (e.g., separate queues for x, y, and z in a 3D grid).

This repository focuses on the graph-based approach using geographic coordinates.

## Breadth First Search (BFS)

Breadth First Search is an algorithm used to find the shortest path in terms of *number of edges* rather than physical distance or travel time. BFS explores all nodes at a certain distance before moving deeper, which guarantees that the first time the target node is reached, the path found is the minimum-edge path.

BFS is also widely used to solve grid-based problems such as the Flood Fill algorithm because a grid can be treated as an implicit graph, where neighbors are determined by their positions within the grid. Maze-solving is one such example.

While BFS is not used for this geographic dataset (since street distances vary), its principles help frame the other algorithms used in this project.

## Greedy Best-First Search

Greedy Best-First Search assigns each node an estimate of how close it is to the target (typically a straight-line heuristic). At each step, the algorithm expands the node that appears closest to the destination.

This approach usually finds a path quickly and tends to move directly toward the goal. However, it does **not** consider the actual cost required to reach a node. Because Greedy Search cares only about "moving closer," not "moving efficiently," it often produces a path that is not optimal.

It is fast and directionally focused, but not guaranteed to produce the shortest path.

## Uniform Cost Search (UCS)

Uniform Cost Search takes the opposite strategy from Greedy Search. Instead of choosing the node that seems closest to the goal, UCS always expands the cheapest node based on the total cost accumulated from the starting point.

This guarantees that the first time the target is reached, the path is optimal.

However, UCS may explore a large portion of the graph, often moving into areas that are far from the target's direction, due to its cost-driven (rather than direction-driven) nature. This makes UCS optimal but computationally expensive in many cases.

## A* Search

A* Search is a combination of Greedy Search and Uniform Cost Search. It evaluates each node using:

**f(n) = g(n) + h(n)**

where g(n) is the actual cost from the start node and h(n) is the estimated cost to the goal (the heuristic).

This combined approach allows A* to find the optimal path while expanding significantly fewer nodes than UCS. By balancing actual cost with estimated distance to the goal, A* focuses its search in promising directions without sacrificing optimality.

A* is currently the most widely used algorithm in navigation systems, robotics, and game AI due to its efficiency and guarantee of finding the shortest path.

## Experimental Results

Both Greedy Best-First Search and Uniform Cost Search were tested on real Indiana road data consisting of 31,860 nodes. The test route traveled from southern Indianapolis (39.640074, -86.134052) to northern Indianapolis (39.7477522, -86.1763304), approximately 12 km straight-line distance.

### Uniform Cost Search Results

- **Path length:** 9.26 km
- **Road segments:** 71
- **Key roads:** South East Street, Madison Avenue, South Meridian Street, South White River Parkway

UCS found the mathematically optimal path by thoroughly exploring nodes based on accumulated distance.

### Greedy Best-First Search Results

- **Path length:** 11.54 km
- **Road segments:** 81
- **Key roads:** South Meridian Street, Bluff Road

Greedy Search found a valid path but one that was **24.6% longer** than the optimal path.

### Analysis

The results demonstrate the fundamental trade-off between these algorithms. Uniform Cost Search guarantees the shortest path through comprehensive exploration. It correctly identified that following South East Street northward before turning west was more efficient than more direct-looking routes.

Greedy Best-First Search likely explored fewer nodes by focusing on the goal direction but made suboptimal routing decisions. The algorithm selected roads like South Meridian Street and Bluff Road that appeared to move directly toward the goal but resulted in a longer total distance due to the actual road network structure.

The 24.6% difference in path length illustrates that in real road networks, the most direct-looking route is not always the shortest. Roads curve, change direction, and have varying distances that make the optimal path non-obvious without proper cost consideration.

## Conclusion

This project demonstrates the practical differences between Greedy Best-First Search and Uniform Cost Search on real geographic data. While Greedy Search can find paths quickly by heading toward the goal, Uniform Cost Search finds shorter paths through thorough exploration. For applications requiring optimal paths, UCS or A* should be used. For applications where speed matters more than optimality, Greedy Search remains a viable option. A* Search represents the best of both worlds, combining the efficiency of heuristic-guided search with the optimality guarantees of cost-based exploration.