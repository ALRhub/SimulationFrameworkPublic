# 8. Gym Guide

The ALR SimulationFramework offers two possibilities of creating a robotic manipulation task. The first one is by using the provided classes in the [alr_sim](../alr_sim). Here useful classes for the Scene, Robot and Cameras are provided, so that you do not need to know all the internals of the different physic simulators. You can find examples in the [demos](../demos) directory. Please refer to the [Classic Task Guide](04_classic_task.md) for more details.

The second option is to design an OpenAI Gym environment. Gym environments are popular environments to benchmark reinforcement learning algorithms and are commonly used in the RL community. This guide tries to help you through this process.

## 8.1 Create your own Gym Environment

 The key idea of Gym is to provide a standardized interface between reinforcement learning algorithms and environments 
 in which the agent is acting. For information on how to get started with Gym see 
 https://gym.openai.com/docs/.
 
 To set up your own environment using the provided simulation framework follow the steps below:
- Set up your scene by specifying the different objects you want to have in the simulation. For information how to 
configure a .xml file see http://www.mujoco.org/book/XMLreference.html
- Create a class which inherits from the ```MujocoEnv``` class and implement the following methods:
    
    1. ```environment_observations()```: Returns all the observations from the environment. For example object
     positions, velocities etc ...
    2. ```_reward()```: Implement the reward function here.
    3. ```callback_randomize_env()```: To randomize your environment after each reset, you can specify the details
     here. 
    If the environment is deterministic, return ```None```.
    4. ```_termination()``` (optional): Define a termination condition for your environment. If the condition is
     satisfied, the environment resets.


For an example see ```pick_and_place_env.py```.

[Back to Overview](./)