# Optimization based Receding Horizon Trajectory Planning using Bernstein Polynomials
## Robot Motion Planning and Optimal Control using Non-linear Programming

The goal of this project was to design and implement a trajectory optimization algorithm for long and/or complex trajectories requiring a high order of approximation.

**Long and/or complex trajectories:**

1) Require a high order of approximation
2) Require high computational power to manipulate (optimize)
3) Computational complexity increases with the order of the system

**The receding horizon optimization scheme brings the following advantages:**

1) Breaks the problem down in smaller, more manageable pieces using a sliding time window
2) Decreases computational complexity
3) Enables inclusion of dynamic obstacles in trajectory planning

**We further employ Bernstein Polynomials for trajectory optimization for the following reasons:** 

1) Convex hull property
2) Degree elevation property
3) De Casteljau Algorithm



