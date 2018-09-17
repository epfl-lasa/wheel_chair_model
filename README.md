# quickie_salsa_m2
The gazebo model of the quickie salsa m2 wheelchair

**NOTE: THIS IS NOT THE FINAL REPO AND NOT COMPLETE. THIS NOTE WILL BE REMOVED ONCE WE HAVE THE WORKING/VERIFIABLE MODEL.**

## Velocity Controller for Obstacle Avoidance
Run the velocity controller with the attractor at the origin an one obstacle: 
```
rosrun wheel_chair_model velocity_controller.py
```
Or with specification of an attractor away from the origin
```
rosrun wheel_chair_model velocity_controller.py <xPos_attractor> <yPos_attractor> <number of obstacles>
rosrun wheel_chair_model velocity_controller.py 0 0 1
```

The obstacles are all initialized in a circle form. It takes them a bit of time to take a random-like shape. Therefore, during simulation only the attracter should be changed at runtime by publishing a Pose2D to the topic '/Atteactor_position' topic.
```
rostopic pub /Attractor_position geometry_msgs/Pose2D "x: 0
y: 0.0
theta: 0.0"
```

code
# Acknowledgment
The .sdlr file is prepared by Bonolo Mathibela <b.mathibela@ucl.ac.uk> 
