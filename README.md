# Robotics-final-Project

This is the final project for my robotics programming class; part of the robotics and autonomous systems minor. <br/>
The assignment was to write an algorithm for the turtlebot3 robot to be able to solve any maze. <br/><br/>
The assignment has two parts:  <br/>
the simulation code that uses Gazebo to visualize our algorithm and the hardware run where our final algorithm was given 3 attempts to solve a randomized maze. <br/><br/>
Simulation Run Video: https://www.youtube.com/watch?v=aHvKMaxJv4s <br/>
In-person Hardware assessment: https://www.youtube.com/watch?v=JrBf_GEsA6Y <br/> <br/>
TO RUN: <br/> 
Hardware: ros2 run maze_hardware maze <br/>
Simulation: launch world: ros2 launch maze_simulation tb3_world_2.launch.py <br/>
&emsp; &emsp;&emsp;&emsp;&emsp;run algorithm: ros2 run maze_simulation turntest 
