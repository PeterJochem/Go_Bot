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

## Connect to the Container  

## Setting up the Docker Container 
 
1) ```sudo docker pull osrf/ros:melodic-desktop-full``` <br />
The standard Docker Hub ROS images are the non-Desktop ones. These will not install neccsery graphics packages in order to run RVIZ

2) ```sudo docker run -it --env DISPLAY=unix$DISPLAY --privileged  --volume /tmp/.X11-unix:/tmp/.X11-unix -dt --name robot_env --privileged -v /dev/ttyDXL/:/dev/ttyDXL --restart unless-stopped -v `pwd`:/root/workspace osrf/ros:melodic-desktop-full```

3) ```sudo docker exec -it robot_env bash``` <br />

4) ```source ros_entrypoint.sh``` <br />

5) ```sudo apt-get update``` <br />
Without this, rosdep won’t find any packages with the given names to install in the docker container

6) ```mkdir /etc/udev``` <br /> 

7) ```sudo apt install udev``` <br />

8) ```rosdep update``` <br /> 
 
9) ```rosdep install --from-paths src --ignore-src -r -y``` <br />

10) ```sudo apt install python-pip``` <br />

11) ```sudo pip install modern_robotics``` <br />

12) ```sudo apt install vim``` <br />

	
