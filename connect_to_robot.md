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
cd ~/Code/franka-lightweight-interface
bash run-rt.sh
franka_lightweight_interface 17 panda_ --sensitivity low --joint-damping off
```
Note that the franka_id=17 (in this case) changes depending on the robot, by default it's 16

4. Run Optitrack
``` sh
cd ~/Code/CITRIFIED/optitrack
bash docker-run.sh
```
#### Sanity check: 1 obstacle == 36 bytes

5. 
Source environment:
``` sh
source ~/.profile_lukas
```
the information message `access control disabled, clients can connect from any host` should appear. Warning, this converts the CapsLock into a CTRL key.

Go to franka_obstacle_avoidance directory and run docker:
``` sh
cd ~/Code/franka_obstacle_avoidance
bash run-docker.sh
```

In the docker-terminal run:
Run lunch file:
```sh
ros2 launch launch/franka.launch.py
```

Split the window (`CTRL+b "`: horizontal / `CTRL+b %`: vertical) and switch windows (`CTRL+b o`) / (`CTRL+b [ARROW KEY]`). 
```sh
[run your commands]
````

e.g.
```sh
python scripts/nonlinear_avoider.py
```

To have a (compliant) control storer:
``` sh
python franka_avoidance/control_repeater.py
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


## Run and recompile robot:
Make sure the control library is copied to `/home/ros/control_libraries` in the `docker-run.sh`. Recompile with:
``` bash
sudo ./control-libraries/source/install.sh -y && sudo pip3 install ./control-libraries/python
```


# Idle Mode
Terminal 1
cd ~/Documents/LASA/learning-user-safety
aica-docker interactive learning-safety-margin:noetic -u ros --net host --no-hostname -v data_vol:/home/ros/ros_ws/src/learning_safety_margin/data
roslaunch learning_safety_margin demo.launch demo:=idle_control

Terminal 2
```
cd ~/Documents/LASA/franka-lightweight-interface
bash run-rt.sh 
franka_lightweight_interface 16 panda_ --sensitivity low --joint-damping off
```