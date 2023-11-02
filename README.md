# ALR SimulationFramework

Maintained by the ALR team at KIT, this Python framework can be used to simulate robotic tasks for scientific research and teaching. Our default environment consists of a Franka Panda robot model.

The Franka Panda robot, is a robotic arm with 7 degrees of freedom, 1KHz control, torque-sensing in all joints, access to multiple research apps and full ROS capability. Franka Panda is therefore widely used in scientific research, including robot manipulation, machine learning etc. For more information, please visit: [Franka Panda](https://www.franka.de/technology)

We currently support the following physic engines, which can be used interchangebly:
- [PyBullet](https://github.com/bulletphysics/bullet3)
- [Mujoco](http://www.mujoco.org/)

Additionally we provide some environments optimized for reinforcement learning. They follow the  [OpenAI Gym](https://gym.openai.com/) interface and support the deployment of [Stable Baselines](https://stable-baselines.readthedocs.io/en/master/) models.

## System Requirements
This simulation framework has been tested in the following operating systems:
- Ubuntu 18.04, 20.04
- Mac OS

Ubuntu 22.04 currently only works in Xorg Mode, for a small tutorial on how to switch to Xorg from Wayland look [here](https://beebom.com/how-switch-between-wayland-xorg-ubuntu/)

## Installation
Due to the various dependencies of the physics engines, we recommend installing the ALR Simulationframework in a Anaconda/Miniconda environment. Please follow the steps as described in our detailed [Installation Guide](doc/01_installation.md)

## Style guidelines
Please make sure that your code conforms to **flake8** conventions. You can automatically check and correct all requirements by using
[pre-commit](https://pre-commit.com/). Follow the installation instructions and run **pre-commit run --all-files** to verify your installation. Before
each commit, all necessary style guides are checked and, if possible, automatically corrected.

## Quickstart Guide
We strongly recommend you familiarize yourself with some of the ALR Simulationframework's core concepts.
Please refer the [Guide Chapters](doc/README.md) marked as **[recommended]**.

## Helpful Resources
Physic engines:
- Pybullet
  - [PyBullet](https://github.com/bulletphysics/bullet3)
  - [Pybullet's Official Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3)
- Mujoco
  - [Mujoco](http://www.mujoco.org/)
  - [Mujoco C++ API](http://www.mujoco.org/book/APIreference.html)
  - [Mujoco-Py Python Bindings](https://github.com/openai/mujoco-py)
  - [Mujoco XML Modeling Schema](http://www.mujoco.org/book/XMLreference.html)

Gyms & Reinforcement Learning:
- [OpenAI Gym](https://gym.openai.com/)
- [Stable Baselines](https://stable-baselines.readthedocs.io/en/master/)
