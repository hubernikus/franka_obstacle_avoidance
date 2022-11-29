# franka_obstacle_avoidance

Following repositories are needed, best to store them in one root folder; we'll refer to it as 
`$franka`-folder.

## Build this repo!
Build docker file:
``` bash
cd simulator-backend/pybullet_zmq
bash docker-build.sh
```

### Run the simulator
Build docker file:
``` bash
bash docker-build.sh
```

## Simulator setup
The repository can be found under
``` bash
git clone --recurse-submodules https://github.com/hubernikus/simulator-backend.git
```

Build docker file:
``` bash
cd simulator-backend/pybullet_zmq
bash docker-build.sh
```

Run docker file:
``` bash
bash docker-run.sh
```

## Optitrack (Camera for Obstacle Recognition on Real Robot)
The repository can be found under
``` bash
git clone https://github.com/epfl-lasa/CITRIFIED.git
```

Build docker file:
``` bash
cd optitrack
bash docker-build.sh
```

Run docker file (obstacles are now detectable):
``` bash
bash docker-run.sh
```

## The main-simulator environment
The repository can be found under
``` bash
git clone https://github.com/hubernikus/franka_obstacle_avoidance
```

Build docker file:
``` bash
cd franka_obstacle_avoidance
bash docker-build.sh
```

Run docker file (obstacles are now detectable):
``` bash
bash docker-run.sh
```
