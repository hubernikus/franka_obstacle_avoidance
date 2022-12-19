# Robot Connection
The instructions how to connect to the robot are described bellow.

1. Turn on robot [franka]

2. Turn on web-interface
```
firefox https://172.16.0.2/desk/
```
PW: frankafranka
Click the 'unlock' button (middle-right)

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
4. Run whatever you need to (!)
Go to franka_obstacle_avoidance directory and run docker:
``` sh
cd Code/lukas/franka_obstacle_avoidance
bash docker-run.sh
rviz2 rviz/franka_obstacle.rviz
```

Split the window (`CTRL+b "`) and switch windows (`CTRL+b o`):
```sh
[run you commands]
````

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