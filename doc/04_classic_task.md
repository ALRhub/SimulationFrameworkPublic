# 4. From Demo to Your Task
- [4. From Demo to Your Task](#4-from-demo-to-your-task)
	- [4.1. Design Your Task](#41-design-your-task)
	- [4.2. General Workflow for the Classic Interface](#42-general-workflow-for-the-classic-interface)
	- [4.3. Choose Engine](#43-choose-engine)
	- [4.4. Joint vs. Task Space Controller](#44-joint-vs-task-space-controller)
		- [4.4.1. Mujoco Mocap / IK](#441-mujoco-mocap--ik)

The ALR SimulationFramework offers two possibilities of creating a robotic manipulation task. The first one is by using the provided classes in the [alr_sim](../alr_sim). Here useful classes for the Scene, Robot and Cameras are provided, so that you do not need to know all the internals of the different physic simulators. You can find examples in the [demos](../demos) directory. This guide tries to help you through this process.

The second option is to design an OpenAI Gym environment. Gym environments are popular environments to benchmark reinforcement learning algorithms and are commonly used in the RL community. Please refer to the [Gym Guide](07_gym.md) for more details.

## 4.1. Design Your Task
Before start writing your own task, you should think about your requirement, such as: 

- What kind of Robot does this task require? Multiple independent robots, a singular robot with one arm? Two arms? Something different?
- What kind of objects does my task require and how many?
- How should I control the robot?
- Do I need to quickly deploy and benchmark RL algorithms?
- Which physics engine should I use?

## 4.2. General Workflow for the Classic Interface

The demos in this repository follow a similar procedure:
1. Select a physics engine.
2. Create all needed objects and cameras.
3. Instantiate a Robot
4. Create a Scene loading the Robot and the objects.
5. Control the Robot in either Joint Space or Task Space (cartesian space) to finish a simple task.

## 4.3. Choose Engine

- Mujoco

	- All the scripts in [Mujoco_Demos](../demos/mujoco) are using Mujoco as physics engine
	- You may find that all the demos here load the objects before create a scene. Internally, Mujoco parses an .xml model on startup. Therefore, all objects need to be loaded before the Scene is started.
	- Mujoco is slower than pybullet, but it is considered to be more realistic in its physics computations, especially for robotic grasping tasks.

- PyBullet

	- All the scripts in [PyBullet_Demos](../demos/pybullet) are using PyBullet as  physics engine. 
	- In contrast to Mujoco, here in PyBullet, we can add object to the simulation dynamically.
	- Pybullet performs faster than Mujoco, but is considered to be a bit more simplified with respect to its physics simulation.

## 4.4. Joint vs. Task Space Controller

In robotics, we differentiate between two control options for robots:
- Joint Space: We send commands to each joint (motor) of the robot. A robotic arm such as the Franka Panda robot we use in the ALR SimulationFramework and our real-world robotic lab has 7 Degrees of Freedom (DoF). Therefore, any command send to the robot is 7 dimensional.
- Task Space: Sometimes also called cartesian space. Here we define a goal coordinate in XYZ world coordinates that the robot should reach. On a real robot, the XYZ cartesian coordinate is translated into one of the corresponding joint configurations. This translation is called "Inverse Kinematics".

In the ALR SimulationFramework, the robot is controlled with joint torques. However, the interface provides functions, so that the user does not need to calculate the required torques. Instead, the user can define a target position for each joint, which the robot tries to drive to (e.g., [Joint Position Demo](../demos/mujoco/robot_control_scene_creation/Demo_Desired_Joint_Position.py)).

Internally, a spline curve is planned. The robot follows this trajectory with simple PD-controllers which compute the required joint torques.

Similarly, the user can give their commands as cartesian XYZ coordinates and WXYZ quaternion orientations. These will be translated into the corresponding joint configurations using Inverse Kinematics (e.g, [IK Demo](../demos/mujoco/robot_control_scene_creation/Demo_IK.py)).

### 4.4.1. Mujoco Mocap / IK
With our new updates, the IK controllers are stable in all settings. While the MujocoMocapRobot is currently broken, we might update it in the future for its intendet mocap purpose, e.g. VR Control.

~~Our IK implementation in Mujoco is not 100% stable right now. If you want to use Task Space commands, you can use the "Mocap Controller" as shown in [Mocap Demo](../demos/mujoco/robot_control_scene_creation/Demo_Mocap.py). Please note that it is not possible to switch from torque-based control to mocap control or vice-versa during simulation.~~

~~The mocap control moves only the end-effector of the robotic arm. All the other joints just follow the movement as if they were attached in a chain. The mocap controller is very fast and precise when you want to control the robot in task space, but you lose some realism. Especially the "elbow" joint might behave strangely, as it does not have any motor force on its own and is just a piece of the chain.~~


[Back to Overview](./)