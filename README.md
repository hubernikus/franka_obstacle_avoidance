# franka_obstacle_avoidance

Following repositories are needed, best to store them in one root folder; we'll refer to it as 
`$franka`-folder.

## Simulator set-up
The repository can be found under
``` bash
git clone https://github.com/epfl-lasa/simulator-backend/tree/main/pybullet_zmq
```
Build docker file:
``` bash
cd simulator-backend/pybullet_zmq
bash docker-build.sh
```

### Run the simulator
Build docker file:
``` bash
bash docker-run.sh
```

## Build this (!) repo!
