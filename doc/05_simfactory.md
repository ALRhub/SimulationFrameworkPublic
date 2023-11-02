# 5. Simulator Building Blocks

- [5. Simulator Building Blocks](#5-simulator-building-blocks)
  - [5.1. SimRepository](#51-simrepository)
  - [5.2. SimFactory](#52-simfactory)
  - [5.3. On the topic of Simulation Objects](#53-on-the-topic-of-simulation-objects)

If you have browsed some of the various [demos](../demos), you have seen that all the demo files begin with a similar code section:

```python
from alr_sim.sims.SimFactory import SimRepository

### Create Robot and Scene. REQUIRED!
sim_factory = SimRepository.get_factory('pybullet')
pb_robot = sim_factory.create_robot()
scene = sim_factory.create_scene(pb_robot, object_list=object_list)
scene.start()
```

The advantage of this approach is that one can quickly swap out the simulators by changing the identifier in the `.get_factory()` line. The simulator internals remain hidden.

In this guide we want to deconstruct and explain this code section, and how it creates a Robot and Scene object.

## 5.1. SimRepository
The `SimRepository` object is a global registry for all the different physics engines (simulators) which the ALR SimulationFramework currently supports.
you can use `SimRepository.list_all_sims()` to query which simulators are supported AND have been installed on your system. The list of supported simulators include to date (07/2021):

- Main Simulators
  - `pybullet` - the [pybullet simulator](https://pybullet.org/wordpress/)
  - `mujoco` - the [MuJoCo simulator](http://www.mujoco.org/)
- "Derived" Simulators
  - `mujoco_mocap` - a MuJoCo simulation, but with the [Mocap Control](04_classic_task.md#441-mujcoco-mocap--ik) option for the robot.
  - `interactive` - a MuJoCo simulation, but you can directly control the robot in real-time using an XBOX Controller. Should be considered experimental.

You can use these string identifiers to retrieve the corresponding `SimFactory` object from the `SimRepository`.

## 5.2. SimFactory
The `SimFactory` objects provide an abstraction layer for the creation of the Simulator specific building blocks:
- a simulator specific [Scene](../alr_sim/core/Scene.py) implementation
- a simulator specific [Robot](../alr_sim/core/Robot.py) implementation
- a simulator specific [Camera](../alr_sim/core/Camera.py) implementation

To get an overview of their public functions, it might be worth to take a look at the base classes in the [core](../alr_sim/core) package.
  

## 5.3. On the topic of Simulation Objects
It is not possible for us to provide a similar abstraction for creating arbitrary objects for a scene. Each simulator has vastly different approaches on how object properties such as visual shapes, collision shapes or even multi-body systems are defined.

In the [PrimitiveObjects](../alr_sim/sims/universal_sim/PrimitiveObjects.py) module, we provide you with primitive shapes such as Boxes, Spheres and Cylinders. If you look at the code, you can see that they implement three interfaces:

```python
class PrimitiveObject(SimObject, PybulletLoadable, MujocoLoadable):
  ...
```
The [SimObject](../alr_sim/core/SimObject.py) interface makes it accessible to read the object states such as position and orientation from the simulations.

The [PybulletLoadable](../alr_sim/sims/pybullet/PybulletLoadable.py) interface defines the function, which the [PybulletScene](../alr_sim/sims/pybullet/PyBulletScene.py) will call to load it into the scene.

The [MujocoLoadable](../alr_sim/sims/mujoco/MujocoLoadable.py) interface defines the function, which the [MujocoScene](../alr_sim/sims/mujoco/MujocoLoadable.py.py) will call to load it into the scene.

When you look into the code for the [PrimitiveObject](../alr_sim/sims/universal_sim/PrimitiveObjects.py) you will see, why it is not possible for us to have a truly universal way to load objects into our scenes without defining a new kind of meta layer for each simulator.


[Back to Overview](./)