# 11. Utils Module
As with many growing software development projects, we also bundle various utility functions in a `utils` package. In this chapter we want to introduce you to some of the most useful functions. Please note, that these might change in future versions.

- [11. Utils Module](#11-utils-module)
  - [11.1. Sim_Path](#111-sim_path)
  - [11.2. Geometric Transformations](#112-geometric-transformations)
  - [11.3. Point Clouds](#113-point-clouds)
  - [11.4. Experimental: Gym Multiprocessing](#114-experimental-gym-multiprocessing)

## 11.1. Sim_Path
[sim_path.py](../alr_sim/utils/sim_path.py) provides a wrapper around python's `os.path.join()` function. You can call this module to create absolute paths from paths relative to the ALR SimulationFramework toplevel directory, or other path needs.

```python
sim_path.sim_framework_path('demos', 'mujoco', 'kitbash')
>>> ...SimulationFramework/demos/mujoco/kitbash/

sim_path.sim_framework_path('./envs/data/duck.obj')
>>> ...SimulationFramework/envs/data/duck.obj

sim_path.sim_framework_path('/home/user', 'documents', 'data.txt')
>>> /home/user/documents/data.txt
```

## 11.2. Geometric Transformations
[geometric_transformation.py](../alr_sim/utils/geometric_transformation.py) provides several functions for various conversions, especially with respect to rotations, e.g. euler-angle to rotation matrix `euler2mat()`

Additionally, you can also find [Quaternion](https://en.wikipedia.org/wiki/Quaternion) specific calculation functions in this module.

## 11.3. Point Clouds
[point_clouds.py](../alr_sim/utils/point_clouds.py) provides a helper class to visualize 3D point clouds in an interactive Open3D viewer.

## 11.4. Experimental: Gym Multiprocessing
The [gym_utils.multiprocessing](../alr_sim/gym/gym_utils/multiprocessing) package provides some experimental functionality to parallelize our gym environments using multiprocessing. While this has been tested in a proof of concept, it must still be considered experimental. But it might provide a good starting point, or even ready-to-go wrapper with minimal adaptions to parallelize your gym environments and speed up your reinforcement learning task.

[Back to Overview](./)