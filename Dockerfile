ARG ROS_VERSION=humble
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}

# upgrade ament_cmake_python
RUN sudo apt update
RUN sudo apt install -y ros-${ROS_DISTRO}-ament-cmake-python

# INSTALL NECESSARY PACKAGES
RUN sudo apt install -y tmux
RUN sudo apt install -y vim-python-jedi
RUN sudo apt install nano 
RUN sudo apt install python-is-python3 -y
RUN sudo apt clean

# Install network inferface [zmq]
WORKDIR /tmp
RUN git clone -b v1.1.0 --depth 1 https://github.com/aica-technology/network-interfaces.git && \
    cd network-interfaces && sudo bash install.sh --auto --no-cpp
RUN rm -rf /tmp/network-interfaces

RUN mkdir -p ${HOME}/lib
WORKDIR ${HOME}/lib

# Dynamic Obstacle Avoidance Library [Only minor changes]
RUN git clone -b main --single-branch https://github.com/epfl-lasa/dynamic_obstacle_avoidance
RUN  python3 -m pip install -r dynamic_obstacle_avoidance/requirements.txt
RUN cd dynamic_obstacle_avoidance && sudo python3 -m pip install --editable .

# Various Tools Library
RUN git clone -b main --single-branch https://github.com/hubernikus/various_tools
RUN python3 -m pip install -r various_tools/requirements.txt
RUN cd various_tools && sudo python3 -m pip install --editable .

# Semester-Project-Learning

# Semester-Project-Avoiding
RUN git clone -b main --single-branch https://github.com/TicaGit/semester_project_LASA_trinca.git
# RUN python3 -m pip install -r requirements.txt
RUN cd semester_project_LASA_trinca && sudo python3 -m pip install --editable .

# Additional Python-Environment
# RUN pip install beautifulsoup4 lxml

# Files are copied indivually to allow compatibility
# for combo and without docker container
# This should be changed for production
RUN mkdir -p /home/${USER}/ros2_ws/src/franka_obstacle_avoidance
WORKDIR /home/${USER}/ros2_ws/src/franka_obstacle_avoidance

COPY --chown=${USER} ../examples .
COPY --chown=${USER} requirements.txt requirements.txt
# COPY --chown=${USER} ../CMakeLists.txt ../package.xml .
# COPY --chown=${USER} ../launch ./launch
# COPY --chown=${USER} ../rviz ./rviz
# COPY --chown=${USER} ../include ./include
# COPY --chown=${USER} ../scripts ./scripts
# COPY --chown=${USER} ../src ./src
# COPY --chown=${USER} ../combined_approach ./combined_approach


WORKDIR /home/${USER}/ros2_ws/
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"

ENTRYPOINT tmux
