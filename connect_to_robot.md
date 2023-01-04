# Robot Connection
The instructions how to connect to the robot are described bellow.

1. Turn on robot [franka]

2. Turn on web-interface
https://172.16.0.2/desk/
```
firefox https://172.16.0.2/desk/
```
username : frankafranka
PW: frankafranka
Click the 'unlock' button (middle-right)
make sure light is white, otherwise press black button 

3. Run franka lightweight docker.
```sh
cd ~/Documents/LASA/franka-lightweight-interface
bash run-rt.sh
franka_lightweight_interface 16 panda_ --sensitivity low --joint-damping off
```

4. Run Optitrack
``` sh
cd ~/Documents/LASA/CITRIFIED/optitrack
bash docker-run.sh
```
4. 
Source environment:
``` sh
source ~/.profile_lukas
```

Go to franka_obstacle_avoidance directory and run docker:
``` sh
cd ~/Code/lukas/franka_obstacle_avoidance
bash docker-run.sh
```

In the docker-terminal run:
Run lunch file:
```sh
ros2 launch launch/franka.launch.py
```

Split the window (`CTRL+b "`) and switch windows (`CTRL+b o`):
```sh
[run your commands]
````

e.g.
```sh
python examples/example_optitrack_rviz.py
```

## Turn the Robot Off
1. Make sure 'emergency lock' is on
2. Go to the web-portal 
```
firefox https://172.16.0.2/desk/
```
> Lock robot (center-right)
> Go to menu (top right) > shutdown robot
3. Turn of switch

## Troubleshooting

If the docker does not want to connect, try running following command on the host (in order to allow the ssh connections):
``` sh
xhost +
```

Robot does not turn on:
> Try press and release of red button.
> Unplug / replug the robot
> Try again
> (Otherwise ask maxime)



# Idle Mode
Terminal 1
cd ~/Documents/LASA/learning-user-safety
aica-docker interactive learning-safety-margin:noetic -u ros --net host --no-hostname -v data_vol:/home/ros/ros_ws/src/learning_safety_margin/data
roslaunch learning_safety_margin demo.launch demo:=idle_control

Terminal 2
cd ~/Documents/LASA/franka-lightweight-interface
bash run-rt.sh 
franka_lightweight_interface 16 panda_ --sensitivity low --joint-damping off