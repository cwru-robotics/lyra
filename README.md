# lyra
ROS interface and visualization tool for the Haptix/iSense hand tracking system.

You need these to run the quadcopter simulation:
~~~
 git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
 git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor
 git clone https://github.com/tu-darmstadt-ros-pkg/hector_localization
 git clone https://github.com/SyrianSpock/realsense_gazebo_plugin
 git clone https://github.com/tu-darmstadt-ros-pkg/hector_models.git
 git clone https://github.com/cwru-robotics/swarm_simulator
 git clone https://github.com/aionrobotics/aion_r1_description.git
 git clone https://github.com/aionrobotics/aion_r1_gazebo.git
 sudo apt-get install ros-melodic-geographic-msgs
 sudo apt-get install ros-melodic-joint-trajectory-controller
~~~

To run the Lyra package itself, you will need
~~~
git clone https://github.com/ros-drivers/leap_motion.git
~~~

This will compile and run without the Leap Motion SDK, although you won't actually be able to use the Leap to control the drone.
