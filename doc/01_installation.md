# 1. Installation Guide

- [1. Installation Guide](#1-installation-guide)
	- [1.1. System Requirement](#11-system-requirement)
	- [1.2. Software Requirements](#12-software-requirements)
	- [1.3. Installation Steps](#13-installation-steps)
		- [1.3.1. Mujoco](#131-mujoco)
		- [1.3.2. ALR SimulationFramework (Automatic)](#132-alr-simulationframework-automatic)
		- [1.3.3. Manual Installation Steps](#133-manual-installation-steps)
	- [1.4. Deinstallation](#14-deinstallation)


## 1.1. System Requirement

- SimulationFramework has been tested with the following operating systems:

	- Ubuntu 18.04, 20.04
	- Mac OS

- Unsupported systems:

	- MS Windows
		- Windows has not been tested by our team.
		- Especially the Mujoco simulation might be lacking on Windows and must be considered highly experimental / unsupported.
		- As a geek in Computer Science, it is time to start your Linux career :D!

## 1.2. Software Requirements

- Before using this repository, you need to install these software packages:
	- [Mujoco](http://www.mujoco.org/), a state of the art physical simulation engine

		- Mujoco is a physics engine written in C++, which is originally developed at the Movement Control Laboratory, University of Washington and has since been bought (and made open-source) by Google's Deepmind. OpenAI implemented a Python interface (mujoco-py) for this engine, which is utilized by our simulation framework. **Since it is tricky to install Mujoco, please follow our install instruction below.**

	- [Anaconda / Miniconda](https://www.anaconda.com/), to handle software dependencies, install and run simulation in a virtual environment. 

## 1.3. Installation Steps
### 1.3.1. Mujoco
Before starting, you should know:
- SimulationFramework can be used with two different versions of Mujoco. If you want the maximum functionality, install both versions of MuJoCo (2.1 and 2.3). Currently, some experiments use MuJoCo 2.1 and some use MuJoCo 2.3.

To install the older MuJoCo version 2.1, you need to follow these instructions:

1. Download Mujoco 2.1. 
   - You can download Mujoco from the official [Mujoco website](https://mujoco.org/download). It is important that you install the correct version, please scroll down until you find version 2.1.
2. Unzip your package and put everything (bin, doc...) under "$HOME/.mujoco/mujoco210/". That's it!

Since Deepmind bought Mujoco, you do not need to acquire a license anymore. It's free!


You should now have the following folder structure in your `$HOME`:
```
.mujoco/  
└── mujoco210  
    ├── bin  
    ├── doc  
    ├── include  
    ├── model  
    └── sample  
```

Note that Mujoco is a C++ library. The corresponding Python bindings for MuJoCo 2.1 are called `mujoco-py` and will be installed by the installation script later. MuJoCo 2.3 will be installed entirely by the installation script, no previous steps are required.

### 1.3.2 Anaconda / Miniconda

Follow the instructions from the official [Installation Guide](https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html). Miniconda is sufficient but you can also choose Anaconda.

### 1.3.3. ALR SimulationFramework (Automatic)

1. Download / Clone this Git Repository


2. Install system dependency (because this cannot be installed through conda unfortunately). If you are using lab PCs in the ALR lab, those dependencies should already be installed system-wide, so you can skip this step.

   	- ```bash
		sudo apt-get install libosmesa6-dev
		```

3. Automatic Installation:
   - In your terminal run the `install.sh` script to automatically install the ALR SimulationFramework and required dependencies in a new conda environment.
   ```bash
   bash install.sh
   ```
4. Activate your virtual environment (`conda activate my_env`) and then execute `pip install mujoco`. This will install MuJoCo version 2.3 as well as its Python bindings.

5. To allow your PC to communicate with SL running on the PCs that controll the Panda robots, you need to install a module called `py_at_broker`. You should install it inside your virtual environment, so make sure that it is activated. To install `py_at_broker`, clone the [repository](https://github.com/ALRhub/py_at_broker) and then execute:
	- ```bash
		cd py_at_broker
		pip install -e python/
		```

6. Ready to go:
   - Now your SimulationFramework is ready, you can run a demo script from the [demo folder](../demos) to see if everything works. For example:  
```python Demo_Pick_and_Place_IK_Control_Mujoco.py```

### 1.3.4. ALR SimulationFramework Manual Installation Steps
If you for some reason need to install things manually, please refer to our [Manual Installation Guide](13_manual_install.md).

## 1.4. Deinstallation
Since everything is installed in your virtual environment, SimulationFramework will not mess up your global system's dependencies. When you decide to delete Simulation framework, just go with:

```conda env remove --name <ENVNAME>```

Afterwards delete the downloaded folders (`.mujoco/` and the downloaded SimulationFramework)

[Back to Overview](./)