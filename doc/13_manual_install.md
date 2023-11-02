# 13. Manual Installation Process
This page documents the dependencies of the ALR SimFramework.

## 13.1. Minimal dependencies
These are the minimal dependencies needed to execute a pybullet simulation.
- pybullet
- pyyaml
- scipy
- opencv
- gin_config
- [pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html). Important: Do not try to install via pip, as this is a different package. Either install through `conda -c conda-forge pinocchio` or systemwide through `sudo apt`
- [pre-commit](https://pre-commit.com/) to enforce style guides for developing

- matplotlib (for plotting)
- gym (for RL gyms)
- for point cloud visualization, best install in this order:
  1. scikit-learn
  2. addict
  3. pandas
  4. plyfile
  5. tqdm
  6. open3d

## 13.2. Additional Mujoco Dependencies (v2.1)
Mujoco requires additional dependencies, which might need to be installed systemwide, such as 
```
sudo apt-get libosmesa6-dev
```
Additionally, you might need to install
- mesalib 
- ~~glfw~~ not required anymore with Mujoco 2.1 as far as we know
- glew 
- patchelf

These libraries can be easily installed via conda.

Finally install the mujoco-py bindings by executing:
```
pip install mujoco-py
```

Afterwards, you must set two environmental variables:
```
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco210/bin:/usr/lib/nvidia
LD_PRELOAD=$LD_PRELOAD:$CONDA_PREFIX/lib/libGLEW.so
```

This can be done by either editing your `.bashrc` file to include the lines:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco210/bin:/usr/lib/nvidia
export LD_PRELOAD=$LD_PRELOAD:$CONDA_PREFIX/lib/libGLEW.so
```

or setting the environmental variables in your conda env only:
```
conda env config vars set LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco210/bin:/usr/lib/nvidia
conda env config vars set LD_PRELOAD=$LD_PRELOAD:$CONDA_PREFIX/lib/libGLEW.so
```

## Installing the ALR Simframework
Right now the SimFramework **must** be installed with the `-e` option:
```
pip install -e .
```

This is because otherwise the assets, such as XML models etc., cannot be found.