# Interactive Control
This package introduces an interactive control option to the ALR SimulationFramework. It is built on the MuJoCo physics simulator and offers real-time robot control using an XBOX controller or an experimental Smartphone Accelerometer controller.

You can access this simulation by using our SimRepository / SimFactory architecture:

```python

from alr_sim.sims.SimFactory import SimRepository

### Create Robot and Scene. REQUIRED!
sim_factory = SimRepository.get_factory('mj_interactive')
ia_robot = sim_factory.create_robot()
scene = sim_factory.create_scene(ia_robot)
scene.start()
```

## Requirements:
```
pip install requests inputs
```

## ia_robots:
The [InteractiveRobot](ia_robots/ia_mujoco_robot.py) overwrites the [MujocoRobot](../MujocoRobot.py)'s `run()` method for a custom "game loop" similar to video games:

1. Read the current robot endeffector configuration
2. read the user inputs from the controller device
3. compute the new, desired endeffector configuration and an appriopriate joint configuration using pybullet kinematics
4. drive the robot to the new joint position.

To perform the inverse kinematics calculation, a pybullet simulation runs in the background. For each iteration of this "game loop", the fictive pybullet robot is set to mirror the current robot state and then used to compute the desired target joint position.

## ia_ctrl
This directory contains the [TCPController](ia_ctrl/TCPController.py) module. This is the class responsible for translating user input to the desired TCP position.

## devices
This directory handles the input reads for the user input devices in real-time using a dedicated thread.
Currently, supported devices include the XBOX controller and smartphone connection using the [Phyphox](https://phyphox.org/)

### Xbox Controller
You can use an Xbox controller to control the robot endeffector.
Here is an overview of the current Control layout:
- Left analog Stick: XY Position
- Right analog stick: XY Orientation
- Left and Right Trigger: Z Orientation
- LB / RB: Z Position
- A/B Gripper Open/Close
- X: Start / Stop Recording
- Select: Plot Recording
- Start: End Simulation

### phyphox
Phyphox is an app by the RWTH Aachen to conduct experiments using a smartphone's sensor. By creating a [custom experiment](https://phyphox.org/editor/) we can read out the phone's aceelerometer and gyroscope, and place some one-screen buttons for controlling the robot grippers.
[This](devices/phyphox_assets/robot_remote.phyphox) XML file can be imported by the phyphox editor for a supported configuration.


Phyphox opens a [webserver](https://phyphox.org/remote-control/) on the phone which sends the sensor readouts using json.

Currently, this controller is more of a proof of concept. Adding a Kalman-Filter should counter-act the sensor drift on the accelerometer, but has not been implemented yet.

### combo
The experimental combo controller combines XBox and Phyphox control. The Phyphox app is used to read the endeffector orientation using the smartphone's gyroscope. All the other inputs are handled by the xbox controller.