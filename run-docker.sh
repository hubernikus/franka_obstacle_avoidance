#!/bin/bash

# it: Do iterative or non-iterative terminal
docker run \
	   -it \
	   -e DISPLAY=$DISPLAY \
	   -h $HOSTNAME \
	   --net host \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   -v $HOME/.Xauthority:/home/ros2/.Xauthority \
	   -v "$(pwd)"/launch:/home/ros2/ros2_ws/src/franka_avoidance/launch:rw \
	   -v "$(pwd)"/config:/home/ros2/config:rw \
	   -v "$(pwd)"/scripts:/home/ros2/ros2_ws/src/franka_avoidance/scripts:rw \
	   -v "$(pwd)"/franka_avoidance:/home/ros2/ros2_ws/src/franka_avoidance/franka_avoidance:rw \
	   -v "$(pwd)"/../various_tools/vartools:/home/ros2/python/various_tools/vartools\
	   -v "$(pwd)"/../dynamic_obstacle_avoidance/dynamic_obstacle_avoidance:/home/ros2/python/dynamic_obstacle_avoidance/dynamic_obstacle_avoidance\
	   -v "$(pwd)"/../dynamic_obstacle_avoidance/tests:/home/ros2/python/dynamic_obstacle_avoidance/tests\
	   -v "$(pwd)"/../nonlinear_obstacle_avoidance/scripts:/home/ros2/python/nonlinear_obstacle_avoidance/scripts:rw \
	   -v "$(pwd)"/../nonlinear_obstacle_avoidance/nonlinear_avoidance:/home/ros2/python/nonlinear_obstacle_avoidance/nonlinear_avoidance:rw \
	   ros2_franka_avoidance

	    # -v "$(pwd)"/../semester_project_LASA_trinca/docker_scripts:/home/ros2/ros2_ws/src/franka_avoidance/project_thibaud/docker_scripts:rw \
	   # -v "$(pwd)"/../semester_project_LASA_trinca/librairies:/home/ros2/ros2_ws/src/franka_avoidance/project_thibaud/librairies:rw \
# Get sourceof ocntrol libary
# -v "$(pwd)"/libs/control-libraries:/home/ros2/control-libraries:rw \

# Project Ekin
# -v "$(pwd)"/../epfl_semester_project_1/examples:/home/ros2/ros2_ws/src/franka_avoidance/project_ekin/examples:rw \
# -v "$(pwd)"/../epfl_semester_project_1/src:/home/ros2/ros2_ws/src/franka_avoidance/project_ekin/src:rw \
# -v "$(pwd)"/../epfl_semester_project_1/shared_data:/home/ros2/ros2_ws/src/franka_avoidance/project_ekin/data:rw \
	   
# -v "$(pwd)"/local/various_tools/vartools:/home/ros2/python/various_tools/vartools:rw \
# -v "$(pwd)"/local/various_tools/vartools:/home/ros2/python/various_tools/vartools:rw \
		
# Alternative mounting?!
# --mount type=bind,source="$(pwd)"/visualization,target=/home/ros/rviz \

# Change user to root
# -u root \

# Copy specify file
# -v "$(pwd)"/docker-rviz/qolo_env.sh:/home/ros/qolo_env.sh\


# Run with specific user
# -u root \


