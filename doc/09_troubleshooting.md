# 9. Troubleshooting
- [9. Troubleshooting](#9-troubleshooting)
  - [9.1. Pinocchio module](#91-pinocchio-module)
  - [9.2. ERROR: GLEW initalization error: Missing GL version](#92-error-glew-initalization-error-missing-gl-version)
  - [9.3 numpy.ndarray size changed](#93-numpyndarray-size-changed)
  - [9.4 ImportError libOpenGL.so](#94-importerror-libopenglso)
  - [9.5 Wayland does not work with GLFW](#95-wayland-does-not-work-with-glfw)
  - [9.6 Mujoco: Rendering / Camera issues](#96-mujoco-rendering--camera-issues)
  - [9.7 Pybullet: Inhand Camera freeze](#97-pybullet-inhand-camera-freeze)


This sections details some known configuration issues and how to solve them. Especially Rendering using OpenGL is prone to crash. When in doubt, please read all the sections. Sometimes the same cause can produce multiple error messages.

## 9.1. Pinocchio module
Unfortunately, two python packages both use the `pinocchio` name. We recommend using conda to install all the dependencies of our SimulationFramework:
```
conda install pinocchio
```
Alternatively, you can install it systemwide following the instructions at https://stack-of-tasks.github.io/pinocchio/download.html

Using `pip` will install the wrong package, causing this error!!


## 9.2. ERROR: GLEW initalization error: Missing GL version
When using mujoco you might encounter this error, when working with the virtual RGB-D cameras.

During setup you had to set the environmental variable `LD_PRELOAD`. To the best of our knowledge, this variable **must** be set when using "windowed" mode (`render=HUMAN`).  
But this variable **must not** be set when using offscreen rendering (`render=OFFSCREEN`). As this will not use OpenGL Hardware rendering, this might be very slow and "blind" mode (skipping rendering with `render=BLIND`) might be preferred.

We currently suspect this is caused by an issue with NVidia-GPU drivers and is outside of our scope to fix.

## 9.3 numpy.ndarray size changed
This error occurs when there is a version mismatch with numpy. Please remove all numpy installations in both pip and conda:

```bash
conda uninstall numpy
pip uninstall numpy
```

Afterwards install numpy only with pip:
```bash
pip install --upgrade numpy
```

## 9.4 ImportError libOpenGL.so
In some instances after following the installation instructions, the mujoco demos might show this behavior and abort with an error: 
```bash
Import error. Trying to rebuild mujoco_py.
running build_ext
building 'mujoco_py.cymj' extension
... (log output pruned) ...
ImportError: libOpenGL.so.0: cannot open shared object file: No such file or directory
```
In this case, OpenGL is missing. It can be installed with
```bash
sudo apt install libopengl0
```

## 9.5 Wayland does not work with GLFW
There is still lacking support for Wayland in GLFW. Switch to X11 or similar alternatives to run the simulation with graphical interface for MuJoCo.

## 9.6 Mujoco: Rendering / Camera issues
Mujoco OpenGL rendering is error prone. Try switching to Mujoco v2.0, which we find to be more stable.

## 9.7 Pybullet: Inhand Camera freeze
Pybullet Camera rendering can sometimes freeze the whole program execution. This reliably happens in a MultiBot Context, but the issue is the `p.getCameraImage()` call. Therefore we suspect that the issue is caused by Pybullet itself and not our implementation.

See also [this Pybullet Forum Thread](https://pybullet.org/Bullet/phpBB3/viewtopic.php?p=43935&hilit=camera#p43935).

[Back to Overview](./)
