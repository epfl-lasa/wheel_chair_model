# Wheel Chair Simulation 

## Installation
Install ignition-math, as described here:
https://ignitionrobotics.org/api/math/6.4/install.html

! For ubuntu 14.04 (trusty with ROS inidgo), install version 3:
```
sudo apt-get install libignition-math-dev
```


## Run the simulator
To run the simulator, one just simply need to run
```
roscore
```
and in a new terminal 
```
roslaunch wheel_chair_model Main.launch
```
There are three differnt scenrios avaibale in this package, check Main.launch for more information.

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

The obstacles are all initialized in a circle form. It takes them a bit of time to take a random-like shape. Therefore, during simulation, only the attracter should be changed at runtime by publishing a Pose2D to the topic '/Atteactor_position' topic.
```
rostopic pub /Attractor_position geometry_msgs/Pose2D "x: 0
y: 0.0
theta: 0.0"
```

# Acknowledgment
The .sdlr file is prepared by Bonolo Mathibela <b.mathibela@ucl.ac.uk> 
