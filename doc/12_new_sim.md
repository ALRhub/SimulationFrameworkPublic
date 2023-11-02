# 12. Integrating New Simulators

- [12. Integrating New Simulators](#12-integrating-new-simulators)
  - [12.0 Attention: Quaternions](#120-attention-quaternions)
  - [12.1. Scene](#121-scene)
  - [12.2. RobotBase](#122-robotbase)
  - [12.3. Camera](#123-camera)
  - [12.4. SimFactory](#124-simfactory)
  - [12.5. Loadable Interface](#125-loadable-interface)

In this chapter we want to describe the required steps to add a new physics engine to the ALR Simulation Framework.
You must implement 4 required interfaces:
- [`Scene`](../alr_sim/core/Scene.py)
- [`RobotBase`](../alr_sim/core/Robots.py)
- [`Camera`](../alr_sim/core/Camera.py)
- [`SimFactory`](../alr_sim/sims/SimFactory.py)

Also, it might be helpful to users to define a new interface `<SIM>Loadable`, similar to [`MujocoLoadable`](../alr_sim/sims/mujoco/MujocoLoadable.py) or [`PybulletLoadable`](../alr_sim/sims/pybullet/PybulletLoadable.py) and use this to extend the [`PrimitiveObjects`](../alr_sim/sims/universal_sim/PrimitiveObjects.py) classes.

## 12.0 Attention: Quaternions
Within the SimFramework, we use the `wxyz` order of defining quaternions. Some simulators, such as pybullet, use `xyzw` instead. Always use the `wxyz` convention, especially for user facing methods. You can use helper functions in [geometric_transformation.py](../alr_sim/utils/geometric_transformation.py) to convert the order when interfacing with your simulator.

## 12.1. Scene
The Scene interface acts like a repository for the simulation. It holds all the loaded objects and surroundings and encapsulates the internal simulation state.

Important things to consider:
- Your simulation should only begin once `scene.start()` is called.
- Does your simulator support instantiation of objects after it has been started?
- Use the [`SimObjectRepository`](../alr_sim/core/sim_object/sim_obj_repo.py) class to keep track of simulated objects by reference, name or sim_id.
  This required steps to load objects and set their ids should occur when adding the objects, or when executing the internal setup function `scene._setup_scene()`.
- Keep in mind, that your simulation scene should supported multiple robots at once, also known as the multi-bot feature.

## 12.2. RobotBase
As the name implies, the RobotBase interface defines the robot. The two most important functions to implement are:

- `receiveState`, in which the robot should ask its simulator what its current state is, especially with respect to its joints.
- `prepareStep`, in which the robot should receive a `target_joint_acc` from its active controller and compute the required command for the sim, e.g. motor torques. The [`MujocoRobot`](../alr_sim/sims/mujoco/MujocoRobot.py) implementation is a good reference implementation.

## 12.3. Camera
The camera must be able to capture images of the current simulated environment.

## 12.4. SimFactory
Implement a SimFactory which acts as a kind of Facade for the various newly implemented interfaces. For more details on its functionality, please refer to [Chapter 5](05_simfactory.md).

Remember to register your SimFactory at the SimRepository, e.g.
```python
"""classic_framework/new_sim/NewSimFactory.py"""
class NewSimFactory:
    ...
    # Class Ends

# On Module Level, after defining the class    
SimRepository.register(NewSimFactory(), 'newsim')

#--------------
"""classic_framework/new_sim/__init__.py"""
import NewSimFactory
```

Having the `import` statement in the packages `__init__.py` file imports the module, when the package is loaded, thus executing the `register()` statement.

Refer to the [Mujoco SimFactory](../alr_sim/sims/mujoco/MujocoFactory.py) and [Mujoco init.py](../alr_sim/sims/mujoco/__init__.py) for a reference implementation.

## 12.5. Loadable Interface
We do not define a global "Loadable" interface class, but it might be helpful for your Simulator to define one on its own. This interface should be used by your Scene implementation to load objects into the simulation.

With this mechanism, simulation objects such as the [`PrimitiveObjects`](../alr_sim/sims/universal_sim/PrimitiveObjects.py) can implement the various Loadable interfaces and can be used in the different simulators without the user needing to adjust their code.

[Back to Overview](./)