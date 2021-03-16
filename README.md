# Flocking Behavior
Implementation of boids with obstacle avoidance

![](Boids.gif)

For this project I implemented an interacting (agent-based) flocking simulation as was discussed by [Craig Reynolds](http://www.red3d.com/cwr/boids/).  It uses operator splitting technique to compute separation, alignment, cohesion and steering accelerations of individual agents, as well as semi-implicit Euler for position update.  Voxel based cache is also being used for efficient neighbor determination.  For environmental interaction, I also included spherical obstacle avoidance method.
