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
	   -v "$(pwd)"/config:/home/ros2/ros2_ws/src/franka_avoidance/config:rw \
	   -v "$(pwd)"/examples:/home/ros2/ros2_ws/src/franka_avoidance/examples:rw \
		ros2_franka_avoidance

# -v "$(pwd)"/../franka_obstacle_avoidance:/home/ros2/ros2_ws/src/franka_avoidance \
	   
	   # -v "$(pwd)"/setup.cfg:/home/ros2/ros2_ws/src/franka_avoidance/setup.cfg \
	#    -v "$(pwd)"/scripts:/home/ros2/ros2_ws/src/franka_avoidance/scripts \
	#    -v "$(pwd)"/franka_avoidance:/home/ros2/ros2_ws/src/franka_avoidance/franka_avoidance \
	#    -v "$(pwd)"/examples:/home/ros2/ros2_ws/src/franka_avoidance/examples:rw \
 	#    -v "$(pwd)"/config:/home/ros2/ros2_ws/src/franka_avoidance/config \
	#    
	#    -v "$(pwd)"/setup.py:/home/ros2/ros2_ws/src/franka_avoidance/setup.py \

# Alternative mounting?!
# --mount type=bind,source="$(pwd)"/visualization,target=/home/ros/rviz \

# Change user to root
# -u root \

# Copy specify file
# -v "$(pwd)"/docker-rviz/qolo_env.sh:/home/ros/qolo_env.sh\


# Run with specific user
# -u root \


