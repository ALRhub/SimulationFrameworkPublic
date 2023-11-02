# 7. Multibot Guide

- [7. Multibot Guide](#7-multibot-guide)
  - [7.1 Multibot Initializaton](#71-multibot-initializaton)
  - [7.2 Multibot Movement Commands](#72-multibot-movement-commands)
  - [7.3 Multibot Task Space Commands.](#73-multibot-task-space-commands)
  - [7.4 Pybullet: Inhand Camera freeze](#74-pybullet-inhand-camera-freeze)

The SimFramework supports the simulation of multiple robotic arms in a single scene. This can be used to simulate bimanual setups or multi-agent cooperation.

Please refer to the [Multibot Demos](../demos/mujoco/multi_bot/) for reference implementations.

## 7.1 Multibot Initializaton
A multibot scene can be intuitively initalized by simply creating multiple robots. Remember to initialize the robots with different base positions.

```python
scene = sim_factory.create_scene()
robot = sim_factory.create_robot(scene)
robot2 = sim_factory.create_robot(scene, base_position=[2.0, 0.0, 0.0])
```

## 7.2 Multibot Movement Commands
By using the standard movement commands, e.g.

```python
print(scene.time_stamp) # 0.0
robot.gotoJointPosition(des_q_1, duration=2)
print(scene.time_stamp) # 2.0
robot2.gotoJointPosition(des_q_2, duration=4)
print(scene.time_stamp) # 6.0
```
the movement commands will be executed _sequentially_.

You can use the new optional `block=False` argument to execute movements simultaneously. The movements do not require to be of the same duration.
```python
print(scene.time_stamp) # 0.0
robot.gotoJointPosition(des_q_1, duration=2, block=False)
print(scene.time_stamp) # 0.0
robot2.gotoJointPosition(des_q_2, duration=4)
print(scene.time_stamp) # 4.0
```

Please keep in mind, that the blocking call will control the program execution regardless of duration.
```python
robot.gotoJointPosition(des_q_1, duration=2, block=False)
robot2.gotoJointPosition(des_q_2, duration=4)

# This will wait for robot2 to finish its movement, as this was a blocking call.
robot.gotoJointPosition(des_q_3, duration=2)
```

## 7.3 Multibot Task Space Commands.
Multibot setups support Task Space (IK) Commands. Per default, we assume that the target cartesian position and orientation are in the global coordinate system, i.e. relative to the world origin / table.

The Simframework will convert it to a local coordinate in the robots specific coordinate system, relative to its base position and orientation.

By setting `global_coord=False`, this conversion will be skipped.

```python
scene = sim_factory.create_scene()
robot_default = sim_factory.create_robot(scene)
robot_forward = sim_factory.create_robot(scene, base_position=[2.0, 0.0, 0.0])
### GLOBAL COORDS
# Moves to global [1, 0, 0] coords
robot_default.gotoCartesianPosition([1, 0, 0]) 

# Moves to global [1, 0, 0] coords
robot_forward.gotoCartesianPosition([1, 0, 0]) 

### LOCAL COORDS
# Moves to global [1, 0, 0] coords, i.e. [1,0,0] relative to the default base [0, 0, 0]
robot_default.gotoCartesianPosition([1, 0, 0], global_coord=False) 

# Moves to global [3, 0, 0] coords, i.e. [1, 0, 0] relative to its base [2, 0, 0]
robot_forward.gotoCartesianPosition([1, 0, 0], global_coord=False) 
```

## 7.4 Pybullet: Inhand Camera freeze
Pybullet Camera rendering can sometimes freeze the whole program execution. This reliably happens in a MultiBot Context, but the issue is the `p.getCameraImage()` call. Therefore we suspect that the issue is caused by Pybullet itself and not our implementation.

See also [this Pybullet Forum Thread](https://pybullet.org/Bullet/phpBB3/viewtopic.php?p=43935&hilit=camera#p43935).