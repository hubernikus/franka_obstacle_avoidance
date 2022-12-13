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
RUN sudo rm -rf /tmp/network-interfaces

# RUN mkdir -p ${HOME}/lib
RUN mkdir ${HOME}/python

# Dynamic Obstacle Avoidance Library [Only minor changes]
WORKDIR ${HOME}/python
RUN git clone -b main --single-branch https://github.com/epfl-lasa/dynamic_obstacle_avoidance
WORKDIR ${HOME}/python/dynamic_obstacle_avoidance
RUN python3 -m pip install -r requirements.txt
RUN python3 -m pip install --editable .

# Various tool
WORKDIR ${HOME}/python
RUN git clone -b main --single-branch https://github.com/hubernikus/various_tools.git
WORKDIR ${HOME}/python/various_tools
RUN python3 -m pip install -r requirements.txt
RUN python3 -m pip install --editable .

# RUN mkdir -p ${HOME}/.ssh
# ADD /home/lukas/.ssh/id_rsa ${HOME}/.ssh/id_rsa
# RUN chmod 600 ${HOME}/.ssh/id_rsa
# RUN ssh-keyscan github.com >> ${HOME}/.ssh/known_hosts

RUN mkdir -p /home/${USER}/ros2_ws/src/franka_obstacle_avoidance

# Semester-Project-Avoiding [Thibaud]
WORKDIR /home/${USER}/ros2_ws/src/franka_obstacle_avoidance/
RUN git clone -b main --single-branch https://github.com/TicaGit/semester_project_LASA_trinca.git project_thibaud
WORKDIR /home/${USER}/ros2_ws/src/franka_obstacle_avoidance/project_thibaud
RUN python3 -m pip install --editable .
# RUN python3 -m pip install -r requirements.txt

# Semester-Project-Learning [Ekin]
WORKDIR /home/${USER}/ros2_ws/src/franka_obstacle_avoidance/
RUN git clone -b main --single-branch https://github.com/MerihEkin/epfl_semester_project_1.git project_ekin

# # Install final
# USER root
# RUN python3 -m pip install pybullet
# USER ${USER}

# USER root
# RUN --mount=type=ssh git clone -b main --single-branch git@github.com:MerihEkin/epfl_semester_project_1.git
# RUN chown -R ${USER}:${USER} epfl_semester_project_1
# USER ${USER}
# WORKDIR ${HOME}/python/epfl_semester_project_1
# RUN python3 -m pip install -r requirements.txt
# RUN cd epfl_semester_project_1 && sudo python3 -m pip install --editable .

# Files are copied indivually to allow compatibility
# for combo and without docker container
# This should be changed for production

RUN sudo pip3 install pybullet
RUN sudo ldconfig

WORKDIR /home/${USER}/ros2_ws/src/franka_obstacle_avoidance
# Copy the local folder
COPY --chown=${USER} src src
COPY --chown=${USER} scripts scripts
COPY --chown=${USER} examples examples
# COPY --chown=${USER} local 
COPY --chown=${USER} requirements.txt requirements.txt
COPY --chown=${USER} setup.py setup.py
# COPY --chown=${USER} setup.cfg setup.cfg
RUN python3 -m pip install -r requirements.txt
RUN python3 -m pip install --editable .

# Delete unnecessary files (somehow this doe now work ?!)
RUN rm -f setup.py requirements.txt

# Install Robot description
WORKDIR /home/${USER}/ros2_ws/src
RUN git clone -b v0.1.0 --single-branch https://github.com/aica-technology/franka_panda_description.git

# Install ros2
WORKDIR /home/${USER}/ros2_ws/src
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"

RUN echo "1"
# Pybullet Setup
WORKDIR ${HOME}
RUN mkdir lib
WORKDIR ${HOME}/lib
RUN git clone -b main --single-branch https://github.com/hubernikus/simulator-backend.git
# RUN git clone -b main --single-branch https://github.com/aica-technology/simulator-backend.git
RUN sudo mv ./simulator-backend/pybullet_zmq ..
RUN sudo mv ./simulator-backend/pybullet_simulation ..

WORKDIR ${HOME}
RUN pip3 install --editable ./pybullet_simulation
RUN pip3 install --editable ./pybullet_zmq
RUN rm -rd lib  

# # Clean image
# RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USER}/ros2_ws/src/franka_obstacle_avoidance
# ENTRYPOINT tmux
ENTRYPOINT tmux new "python3 ~/pybullet_zmq/bin/zmq-simulator" ';' split "bash"




