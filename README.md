# Description
I am building a deep-learning Go agent with a robotic arm to also manipulate the pieces. I am using Keras to train the neural network, MoveIt to do motion planning, and ROS to tie everything together.     

# Results 


# How to Run My Code


# ROS Packages
##go_core 
go_core is a Go engine. It has a deep-learning AI to play against. The agent learns to play go by playing many games against itself. Its learned policy is stored as a nueral network in Keras.

##go_reactor
This package runs nodes to facilitate the ReactorX200 robot arm manipulating Go pieces.

# Docker Instructions
I had some trouble getting all the packages at the correct versions to compile so I made a Docker container. Here are the instructions on how to connect to the Docker container and also, if need be, create a new container 
Put it all in a bash script?

## Connect to the Running Container  
1) ```sudo docker ps``` to see if the container is running

2) ```docker exec -it robot_env bash``` <br /> 

## Setting up the Docker Container 

1) ```sudo xhost +``` <br />
2) ```export DISPLAY=:0.0``` <br />
The first two help setup some sort of graphics dependecy within the Docker container. RVIZ won't be able to run without this 
 
3) ```sudo docker pull osrf/ros:melodic-desktop-full``` <br />
The standard Docker Hub ROS images are the non-Desktop ones. These will not install neccsery graphics packages in order to run RVIZ

4) ```sudo docker run -it --env DISPLAY=unix$DISPLAY --privileged  --volume /tmp/.X11-unix:/tmp/.X11-unix -dt --name robot_env --privileged -v /dev/ttyDXL/:/dev/ttyDXL --restart unless-stopped -v `pwd`:/root/workspace osrf/ros:melodic-desktop-full``` <br />

5) ```sudo docker exec -it robot_env bash``` <br />

6) ```source ros_entrypoint.sh``` <br />

7) ```sudo apt-get update``` <br />
Without this, rosdep won’t find any packages with the given names to install in the Docker container

8) ```mkdir /etc/udev``` <br /> 

9) ```sudo apt install udev``` <br />

10) ```rosdep update``` <br /> 
 
11) ```rosdep install --from-paths src --ignore-src -r -y``` <br />

12) ```sudo apt install python-pip``` <br />

13) ```sudo pip install modern_robotics``` <br />

14) ```sudo cp catkin_ws/src/interbotix_ros_arms/interbotix_sdk/10-interbotix-udev.rules /etc/udev/rules.d``` <br />

15) ```sudo udevadm control --reload-rules && udevadm trigger``` <br />

16) ```sudo apt install vim``` <br />

17) ```vim usr/share/ignition/fuel_tools/config.yaml``` 
Change the url in the config.yaml file to ```https://api.ignitionrobotics.org```

## Restarting the Container on Reboot
1) ```sudo docker start robot_env```	
