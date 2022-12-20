#!/bin/bash

# it: Do iterative or non-iterative terminal
docker run \
	   -it \
	   -e DISPLAY=$DISPLAY \
	   -h $HOSTNAME \
	   --net host \
	   -v /tmp/.X11-unix:/tmp/.X11-unix \
	   -v $HOME/.Xauthority:/home/ros2/.Xauthority \
		ros2_franka_avoidance

# -v "$(pwd)"/../franka_obstacle_avoidance:/home/ros2/ros2_ws/src/franka_avoidance \
	   
	   # -v "$(pwd)"/setup.cfg:/home/ros2/ros2_ws/src/franka_avoidance/setup.cfg \
	#    -v "$(pwd)"/scripts:/home/ros2/ros2_ws/src/franka_avoidance/scripts \
	#    -v "$(pwd)"/franka_avoidance:/home/ros2/ros2_ws/src/franka_avoidance/franka_avoidance \
	#    -v "$(pwd)"/examples:/home/ros2/ros2_ws/src/franka_avoidance/examples:rw \
 	#    -v "$(pwd)"/config:/home/ros2/ros2_ws/src/franka_avoidance/config \
	#    -v "$(pwd)"/launch:/home/ros2/ros2_ws/src/franka_avoidance/launch \
	#    -v "$(pwd)"/setup.py:/home/ros2/ros2_ws/src/franka_avoidance/setup.py \

# k
# -v $HOME/Code/dynamic_obstacle_avoidance/dynamic_obstacle_avoidance:/home/ros2/python/dynamic_obstacle_avoidance/dynamic_obstacle_avoidance \
# -v $HOME/Code/various_tools/vartools:/home/ros2/python/various_tools/vartools \

# -v "$(pwd)"/local/*:/home/ros2/python/* \
# -v "$(pwd)"/models/*:/home/ros2/models/*\
# -v "$(pwd)"/src/various_tools/vartools:/python/various_tools/vartools\
# -v "$(pwd)"/src/various_tool:/python/fast_obstacle_avoidance/dynamic_obstacle_avoidance\
# -v "$(pwd)"/src/dynamic_obstacle_avoidance/dynamic_obstacle_avoidance:/python/fast_obstacle_avoidance/dynamic_obstacle_avoidance\
# -v "$(pwd)"/scripts:/home/ros/catkin_ws/src/qolo_fast_modulation/scripts\
# -v "$(pwd)"/src/fast_obstacle_avoidance/fast_obstacle_avoidance:/python/fast_obstacle_avoidance/fast_obstacle_avoidance\
# -v "$(pwd)"/src/fast_obstacle_avoidance/scripts:/python/fast_obstacle_avoidance/scripts\

# Alternative mounting?!
# --mount type=bind,source="$(pwd)"/visualization,target=/home/ros/rviz \

# Change user to root
# -u root \

# Copy specify file
# -v "$(pwd)"/docker-rviz/qolo_env.sh:/home/ros/qolo_env.sh\


# Run with specific user
# -u root \


