# Intel Asctec Leader Follower UAV System
## Coordinated control for multi-UAV systems using MOCAP

*This project was a turning point in my life as it made me realize my love for aerial robotics and the fact that I can pull several all-nighters for the cause!*

The goal of this project was to design and implement a system that would simulate multi-robot coordination and autonomous following of a recognizable mobile target for application in search and rescue, perpetrator interception and robot-based transportation.

We designed a leader-follower system consisting of a pair of Intel-Asctec UAVs equipped with reflective markers for motion capture capabilities. Each UAV was equipped with a PID controller onboard and had its designated ground station. The ground station enabled obstacle avoidance and trajectory planning using a gradient descent-based algorithm implemented in C. 

My team and I designed a novel way of coordinating between the two UAVs by modeling the leader as an obstacle to the follower and tweaking the gradient descent algorithm for the distingushable rigid body to be attracted to it, rather than the general repulsive obstacle avoidance which was implemented on all other rigid bodies in the environment. This was a stepping stone into our understanding of multi-UAV planning and control.

[Here](https://www.youtube.com/watch?v=g6mSGPw2xz8) is a link to the final product of our efforts.



