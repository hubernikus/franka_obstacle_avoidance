# franka_obstacle_avoidance

Following repositories are needed, best to store them in one root folder; we'll refer to it as 
`$franka`-folder.

## Build this repo!
Build docker file:
``` bash
cd simulator-backend/pybullet_zmq
bash docker-build.sh
```

Run simulator
``` bash
python3 pybullet_zmq/bin/zmq-simulator
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

### Windows user
The setup for windows is unfortunately a bit more difficult; one option is to follow the descriptions here:
https://dev.to/darksmile92/run-gui-app-in-linux-docker-container-on-windows-host-4kde

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

# Control Libraries
> New Controller Type: 
	- DERIVED_DISSIPATIVE_LINEAR subclass of DISSIPATIVE
	- We are 'override' the the 'compute_dissipative' function
    - We need to update the source file (!)
	- Set 'new parameter' before you 'compute_dissipative'
	- 'controller->set_paramater_value<Eigen::MatrixXd>("damping", damping_value);
	
FORGET EVERYTHING
Use impedenance:
   - set inertia and stiffness to zero
   - and just update damping
   - Command state get acceleration:
   
   - Desired Force & Desired 
   - command_state.set_linear_acceleration()
   - command_state.set_orientation()
Check out:
   - https://github.com/epfl-lasa/control-libraries/blob/main/python/test/controllers/test_impedance.py




