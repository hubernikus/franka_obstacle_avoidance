FROM ros:noetic-ros-base

# install base dependencies
RUN apt-get update \
	&& apt-get install --no-install-recommends -y \
			   autoconf \
			   automake \
			   curl \
			   featherpad \
			   gdb \
			   git \
			   iputils-ping \
			   libboost-all-dev \
			   libtool \
			   mesa-utils \
			   python3-pip \
			   ros-${ROS_DISTRO}-xacro \
			   ros-${ROS_DISTRO}-robot-state-publisher \
			   ros-${ROS_DISTRO}-rviz2 \
			   rsync \
			   software-properties-common \
			   ssh \
			   wget \
	&& rm -rf /var/lib/apt/lists/*

# INSTALL NECESSARY PACKAGES
RUN apt update \
	&& apt install -y \
	tmux \
	vim-python-jedi \
	nano \
	# gnupg2 curl lsb-core \ 
	# libpng16-16 libjpeg-turbo8 libtiff5 \
	# ros-${ROS_DISTRO}-rviz \
	&& apt clean

# Allow matplotlib-plotting
RUN apt-get install -y python3-tk

# Files are currently just copied -> direct access from github could be done (?)
# but this would require (stable) tags
COPY src python

WORKDIR /python/various_tools
RUN python3.9 -m pip install -e .
RUN python3.9 -m pip install -r requirements.txt

WORKDIR /python/dynamic_obstacle_avoidance
RUN python3.9 -m pip install -r requirements.txt
RUN python3.9 -m pip install -e .

WORKDIR /python/fast_obstacle_avoidance
RUN python3.9 -m pip install -r requirements.txt
RUN python3.9 -m pip install -e .

# Resolve few conflicts
RUN python3.9 -m pip install numpy --upgrade
RUN python3.9 -m pip install --upgrade scikit-image

# Create a user called ROS
RUN groupadd -g 1000 ros
RUN useradd -d /home/ros -s /bin/bash -m ros -u 1000 -g 1000

USER ros
ENV HOME /home/ros

RUN mkdir -p ${HOME}/catkin_ws/src/qolo_fast_modulation/scripts

WORKDIR ${HOME}/catkin_ws/src/qolo_fast_modulation
# COPY messages src/messages
COPY requirements.txt requirements.txt
COPY CMakeLists.txt CMakeLists.txt

# Optional: source could be directly downloaded from git (but no local changes possible...)
# The source code is already copied to the python directory
# COPY src src

# Local environment to allow for installation
# RUN python3.9 -m venv env
# RUN source ./env/bin/activate

# COPY scripts scripts
# Set ROS environment -> 
# ENV ROS_MASTER_URI=http://128.179.186.206:11311

# ROS environment for QOLO
# ENV ROS_MASTER_URI=http://128.179.186.206:11311

USER ros
WORKDIR ${HOME}/catkin_ws
# RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# RUN export ROS_MASTER_URI=http://localhost:11311
ENV ROS_MASTER_URI http://192.168.13.110:11311
ENV ROS_IP 192.168.13.120

# WORKDIR ${HOME}
WORKDIR ${HOME}/catkin_ws/src/qolo_fast_modulation/scripts

# COPY docker-rviz/qolo_env.sh ${HOME}/qolo_env.sh 
# CMD tmux

# Run the main controller (basic
# CMD echo "python3.9 controller_laserscan.py"
