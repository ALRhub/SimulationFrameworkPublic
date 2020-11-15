# README of Robotic Simulation Framework

## Get Started

### Introduction

- This repository

	- Maintained by the ALR team at KIT, this Python written framework offers a robotic simulation environment, which defines a Franka Panda robot model with few objects. Two physics engine interfaces (PyBullet and Mujoco) are available in this simulation, which fulfill the requirements of different tasks in scientific research and teaching usage. 

- Physics engines

	- This simulation framework offers interfaces to use PyBullet and Mujoco as physics engines backend. They are obliged to compute the internal physical behaviors in the simulation, including motion, force&torque, collision and friction etc. 
	- The two engines benefit different features and have different licenses. You may need to apply for a proper license for Mujoco, due to its strict license policy.
	- For more information regarding to the license, please visit: [PyBullet](https://github.com/bulletphysics/bullet3), and [Mujoco](https://www.roboti.us/license.html)

- Panda robot

	- This simulation framework implemented a model of Franka Panda robot, which provides researchers with many distinct features including 7 degrees of freedom, 1KHz control, torque-sensing in all joints, access to multiple research apps and full ROS capability. Franka Panda is therefore widely used in scientific research, including robot manipulation, machine learning etc.
	- For more information, please visit: [Franka Panda](https://www.franka.de/technology)

### System Requirement

- This simulation framework has been tested in the following operating systems:

	- Ubuntu 18.04, 20.04
	- Mac OS

- Other systems

	- Windows

		- Mujoco 2.0+ has deprecated its support in Windows and therefore cannot be installed in Windows.
		- Old version Mujoco 1.5 can run in Windows, but using old version may cause unpredictable problems and is thus STRONGLY not recommended.
		- As a geek in Computer Science, it is time to start your Linux career :D!

### Software Requirement

- Before using this repository, you need to install these software:

	- [Git](https://git-scm.com/), as version control
	- [Anaconda](https://www.anaconda.com/), to handle software dependencies, install and run simulation in a virtual environment. Other Conda apps are also fine.
	- [Mujoco](http://www.mujoco.org/), a state of the art physical simulation engine

		- Mujoco is a physics engine written in C++, which is originally developed at the Movement Control Laboratory, University of Washington. OpenAI implemented a Python interface (mujoco-py) for this engine, which is utilized by our simulation framework. Since it is tricky to install Mujoco, please follow our install instruction below. 

### Get Simulation Framework

- Clone repository from Github

	- ```git clone https://github.com/ALRhub/SimulationFrameworkPublic```

- Get Mujoco

	- Before start, you should know

		- Note Mujoco is a C++ library, while mujoco-py is a Python package.
		- You need to firstly download and unzip Mujoco and place it in a desired folder. Later When you install mujoco-py, it will find and install Mujoco together.

	- Download Mujoco

		- Choose [Linux](https://www.roboti.us/download/mujoco200_linux.zip), or for [MacOS](https://www.roboti.us/download/mujoco200_macos.zip)
		- Unzip your package and put everything (bin, doc...) under "$HOME/.mujoco/mujoco200/"

	- Get a Mujoco license

		- You can apply for a free license, if your are a university student. Or you can use the license of our lab, in case of any publication purpose. For more details of mujoco license, click [here](https://www.roboti.us/license.html).
		- Put your downloaded Mujoco license (mjkey.txt) under "$HOME/.mujoco/" AND under "$HOME/.mujoco/mujoco200/bin"
		- Desired folder structure looks like:  
```
.mujoco/  
├── mjkey.txt  
└── mujoco200  
    ├── bin  
    ├── doc  
    ├── include  
    ├── mjkey.txt  
    ├── model  
    └── sample  
```

- Install system dependency (because this cannot be installed through conda unfortunately)

	- ```sudo apt-get install libosmesa6-dev```

- Prepare virtual environment and install everything

	- Install the latest Anaconda3 or other Conda apps

		- For more support, click [Anaconda](https://www.anaconda.com/)

	- Create virtual environment and install dependencies (C++, python)

		- Open a terminal and export the environment variable LD_LIBRARY_PATH
		 with the absolute directory of your mujoco binary. For example
		  (replace DonaldTrump with your username and path!):  
```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/DonaldTrump/.mujoco/mujoco200/bin```

		- Set the current directory to the cloned repository. Then type 
```conda env create -f conda_env.yml```

	- Activate virtual environment

		- ```conda activate SimulationFramework```

			- You will find the env title (base) changes to (SimulationFramework)

	- Add environment variables to your VIRTUAL environment

		- Open a terminal and activate virtual environment (see above).
		- Set LD_LIBRARY_PATH the absolute directory of your mujoco binary.    
For example(replace DonaldTrump with your username!):  
```conda env config vars set LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/DonaldTrump/.mujoco/mujoco200/bin```
		- Set LD_PRELOAD the absolute directory of your GLEW library (should be in your virtural environment).   
For example(replace DonaldTrump with your username and path to Anaconda!):  
```conda env config vars set LD_PRELOAD=$LD_PRELOAD:/home/DonaldTrump/anaconda3/envs/SimulationFramework/lib/libGLEW.so```
		- Reactivate your virtual environment by 
```conda activate SimulationFramework```
		- (Troubleshooting) If you make any mistakes when setting up variables
		, you can delete them, e.g.   
```conda env config vars unset LD_LIBRARY_PATH``` 

	- Install SimultionFramework into Python path

		- Open a terminal, go to your repository, activate your virtual environment
		- Then  
```pip install -e . ```

	- Ready to go

		- Now your SimulationFramework is ready, you can run a demo under [Demo folder](./demos) to see if everything works. For example:  
```python Demo_Pick_and_Place_IK_Control_Mujoco.py```
		- If you want to use IDEs, like Pycharm and VSCode, we recommend you always:

			- Open a terminal and activate the virtual environment
			- Start your IDE from this terminal, otherwise your IDE may lose some environment variables and cannot run the demo.
			- Set the Python interpreter to the one in the Conda virtual environment

	- Uninstall

		- Since everything is installed in your virtual environment, SimulationFramework will not mess up your global system's dependencies. When you decide to delete Simulation framework, just go with 
```conda env remove --name SimulationFramework```  
then everything is gone.

### File Structure

- The repository mainly contains four parts: Classic Framework, Demonstrations, Gym Framework, and Environments.
- Classic Framework

	- In [classic_framework](./classic_framework) you can find basic level programming modules as well as robot control modules.
	- Better to be used if you want to define your task from scratch.

- Demonstrations

	- In [demos](./demos) you can find some examples of how to define a basic robot learning tasks. Understand all the procedures to define a robot, a scene, a camera system as well as different ways to control a robot.

- Gym Framework

	- In [gym_framework](./gym_framework) you can find high level robot Reinforcement Learning modules where you can quickly define your Reinforcement learning tasks and deploy some benchmark RL algorithms.
	- Inspired by OpenAI Gym.

- Environments

	- In [envs](./envs) you can find the geometry, CAD meshes, textures and configuration files for the robot, camera and other objects. You can use them to define your own task.
	- If you want to create and define your own simulation objects, such as obstacles, bananas, please follow the examples there.

## From Demo to Your Task

### Design Your Task

- In principle the framework ships two possibilities of creating a task. The first one is by using the interface defined in the classic framework. You may use the Robot and Scene class which provide 
useful functions to help building up your own simulation scene. The robot you create will help you controlling the robot and reading out important sensor information. Each robot has a logger, which can be used 
for plotting or reading out the sensor information at the end of your script. In demos -> robot_control_scene_creation you can find examples for the usage of the classic framework.
The second possibility the framework offers is the usage of a gym environment. Gym environments are popular environments to benchmark reinforcement learning algorithms and are commonly used
in the reinforcement learning community.  
Before start writing your own task, you should think about your requirement, such as: How many robots and other objects do I need in my simulation? Which engine should I use? Which controller should I use? Do I need to quickly deploy a RL algorithm?
We will now describe roughly the procedure you may create your scenery / environment.

### General Workflow for the Classic Interface

- Basically all the demos in the repository perform similar procedures: create objects and cameras, create a simulation scene containing all these objects, and a robot through the selected physics engine. Then control the robot in either joint space or task space to finish the simple task.

### Choose Engine

- Mujoco

	- All the scripts in [Mujoco_Demos](./demos/mujoco) are using Mujoco as physics engine
	- You may find that all the demos here load the objects before create a scene. Internally, Mujoco needs a .xml file as configuration and what we do here is to simply parse the object list into this .xml file and then create the scene.

- PyBullet

	- All the scripts in [PyBullet_Demos](./demos/pybullet) are using PyBullet as  physics engine. 
	- On the contrast to Mujoco, here in PyBullet, we can add object to the simulation dynamically.

### Joint vs. Task Space Controller

- Classic Framework
    - The robot in the classic framework is controlled with torques. However, the interface provides functions, where the user does not need to care about
      calculating torques. In principle the user simply defines the desired position the robot should go to. Internally a spline is planned either in joint
      or in task space. The robot follows this trajectory with simple PD-controllers such that a map from desiried positions and velocities to torques is
      provided. You can see examples for joint space control in "Demo_Desired_Joint_Position" in the demos folder, where the robot is supposed to drive to a certain position.
      Further demonstrations such as "Demo_Pick_and_Place_IK_Control" show the usage of the desired task space position and orientation, the robot has to drive to.
      Following a desired joint trajectory is also possible. In this case the user can load a desired joint trajectory, where the robot will receive a desired joint
      position in each time-step. The file "Demo_Follow_Trajectory" shows an example on how to use the functionality given in the classic framework.
      
    - Mujoco offers a so called "mocap" controller which corresponds to task space control. However the control commands are internally handled by mujoco itself such that we do not have access to them. Nevertheless, the mocap controller is very fast and precise when you want to control the robot in task space.
      The mocap controller can be chosen when creating the Mujoco Scene (see Demo_Pick_and_Place_IK_Control). Please note that
      it is not possible to switch from torque-based control to mocap control or vice-versa during simulation. That would require to reload the xml-model. 
      Thus, if you already know that you want to control your robot in task space, it would make sense to chose the mocap control mode.
    
- Gym Environment (currently only offered in Mujoco)
    - Different gym environments have different types of controlling a robot. This will highly depend on your use case. We might do direct position control, where we send only the positions as control commands (e.g. joint positions). We might also do pure torque control, where we send torque commands in each step. This is more a design choice. 
      If you want to design your own environment, you need to take care that the xml file you define your robot has the correct actuator models. 
      Further information to gym environment will be mentioned in the next chapters.
       
### Use Camera for Perception

- Mujoco
	
	- Most Mujoco Scenes come with two predefined cameras:
		- `rgbd` - an RGB-D camera on the robot's endeffector
		- `rgbd_cage` - an RGB-D camera in an over-the-shoulder position

	- to add your own camera to a scene, add a [MujocoCamera](./classic_framework/mujoco/mujoco_utils/mujoco_scene_object.py) to your [MujocoScene](./classic_framework/mujoco/MujocoScene.py)
	```python
	cam = MujocoCamera('my_cam')
	scene = MujocoScene(camera_list=[cam])
	```
	
	
	- alternatively define a new `<camera name="my_cam"/>` tag directly in your XML model (see [XML Reference](http://www.mujoco.org/book/XMLreference.html#camera))

	- the [MujocoScene](./classic_framework/mujoco/MujocoScene.py) class provides several functions to obtain data from the cameras 
	```python
	xyz, rgb = scene.get_point_cloud_from_cam(name='my_cam')
	img = scene.get_rgb_image_from_cam(name='rgbd_cage')
	```

- Pybullet
	
	- the `PyBulletScene` class provides several functions to obtain data from the cameras, e.g. 
	`get_point_cloud_from_cam(cam_id)` or `get_rgb_image_from_cam(cam_id)`

### Create your own Gym Environment

 The key idea of Gym is to provide a standardized interface between reinforcement learning algorithms and environments 
 in which the agent is acting. For information on how to get started with Gym see 
 https://gym.openai.com/docs/.
 
 To set up your own environment using the provided simulation framework follow the steps below:
- Setup your scene by specifying the different objects you want to have in the simulation. For information how to 
configure a .xml file see http://www.mujoco.org/book/XMLreference.html
- Create a class which inherits from the ```MujocoEnv``` class and implement the following methods:
    
    1. ```environment_observations()```: Returns all the observations from the environment. For example object
     positions, velocities etc..
    2. ```_reward()```: Implement the reward function here.
    3. ```callback_randomize_env()```: To randomize your environment after each reset, you can specify the details
     here. 
    If the environment is deterministic, return ```None```.
    4. ```_termination()``` (optional): Define a termination condition for your environment. If the condition is
     satisfied, the environment resets.


For an example see ```pick_and_place_env.py```.
### Others

- Utilities

	- [geometric_transformation.py](./classic_framework/utils/geometric_transformation.py) provides several functions for various conversions, e.g. euler-angle to matrix `euler2mat()`
	- [sim_path.py](./classic_framework/utils/sim_path.py) provides a wrapper arround python's `os.path.join()` function. You can call this module to create absolute paths from paths relative to the Simulation Framework toplevel dir, or other path needs.
	```python
	sim_path.sim_framework_path('demos', 'mujoco', 'kitbash')
	>>> ...SimulationFramework/demos/mujoco/kitbash/

	sim_path.sim_framework_path('./envs/data/duck.obj')
	>>> ...SimulationFramework/envs/data/duck.obj

	sim_path.sim_framework_path('/home/user', 'documents', 'data.txt')
	>>> /home/user/documents/data.txt
	```

- Other techniques
    - Physical Parameters:
        - In Mujoco there are a couple of parameters which define the physical behavior of your object. We have created a "best practices" document, which you can follow to get an intuition for the most important parameters:
            https://www.overleaf.com/read/cfwjsrsgyynr
        - Furthermore the Mujoco Documentation http://www.mujoco.org/book/ will give you a very detailed overview of computation models and parameters. 
	- Multi-threading simulation: Not implemented yet
	- Simulation time vs. Elapsed real time (wall-clock time):
	    - In order to calculate the elapsed real time you first need the specify the ```timestep``` attribute in the
	     MuJoCo .xml file or call the equivalent ```set_dt()``` offered by the ```MujocoSceneParser``` class. Once
	      this parameter is set and you created an instance of the simulation ```sim``` you can call ```sim.step
	      ()``` to advance your simulation for ```timestep``` seconds. For example, if you set ```timestep=0.001``` 
	      then you need to call ```sim.step()``` 1000 times to advance your simulation for one second wall clock time
	      . However there is a small catch: If you set the ```nsubsteps``` parameter when creating the sim, then each
	       call of the ```sim.step()``` method will advance the simulation for ```nsubsteps * timestep``` seconds
	       . For example, if you set ```nsubsteps=10``` and ```timestep=0.001``` then you only need the call 
	       ```sim.step()``` 100 times to advance your simulation for one second wall clock time. Note that the
	        ```timestep``` attribute is often used interchangeably with ```dt```.  

