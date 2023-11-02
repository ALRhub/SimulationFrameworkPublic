# 10. Advanced Simulation Concepts
- [10. Advanced Simulation Concepts](#10-advanced-simulation-concepts)
  - [10.1. Helpful Links](#101-helpful-links)
  - [10.2. Custom Meshes](#102-custom-meshes)
    - [10.2.1. Mujoco: Mesh Jittering](#1021-mujoco-mesh-jittering)
  - [10.3. Timesteps:](#103-timesteps)
    - [10.3.1. mujoco-py: nsubsteps](#1031-mujoco-py-nsubsteps)
  - [10.4. Mujoco Advanced Physics Settings](#104-mujoco-advanced-physics-settings)
  - [10.5. Converting .urdf to .xml files](#105-converting-urdf-to-xml-files)
    - [1. Add Mujoco tags](#1-add-mujoco-tags)
    - [2. Check the mesh file format](#2-check-the-mesh-file-format)
    - [3. Conversion](#3-conversion)
    - [4. Test the Model (optional)](#4-test-the-model-optional)


In this chapter we want to present some of the advanced simulation concepts we acquired through experience.

## 10.1. Helpful Links
Every simulator has its own specific strengths and constraints. Please refer to the official documentations to get more detailed insights:

- Pybullet
  - [PyBullet](https://github.com/bulletphysics/bullet3)
  - [Pybullet's Official Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3)
- Mujoco
  - [Mujoco](http://www.mujoco.org/)
  - [Mujoco C++ API](http://www.mujoco.org/book/APIreference.html)
  - [Mujoco-Py Python Bindings](https://github.com/openai/mujoco-py)
  - [Mujoco XML Modeling Schema](http://www.mujoco.org/book/XMLreference.html)

## 10.2. Custom Meshes
Both Mujoco and Pybullet support the use of custom meshed objects. But both simulators will only approximate the 3D mesh using its convex hull for performance reasons.

![Convex Hull Example](https://raw.githubusercontent.com/kmammou/v-hacd/master/doc/chvsacd.png)  
From [V-HACD Project](https://github.com/kmammou/v-hacd)

Any concave sections or holes (e.g. donut shape) will be "ignored" by simulation. To counter this, one must first decompose the mesh into convex parts. This must be done manually in a 3D modeling software like [Blender](https://www.blender.org/). The aforementioned [V-HACD Project](https://github.com/kmammou/v-hacd) is a Blender Plugin which can support you in this task.

### 10.2.1. Mujoco: Mesh Jittering
When using custom meshes in a Mujoco simulation, you might encounter jittering: The objects do not lie still on the table, but move around slightly. See also from the developers this [forum answer](http://www.mujoco.org/forum/index.php?threads/custom-mesh-jittering-in-mujoco-environment-in-openai-gym.3860/).

In summary: Mujoco cannot compute multiple collision points for custom meshes. If decomposed into submeshes, it can detect collisions for each submesh, thus increasing the stability of the simulation.

For concave meshes this should always be done (as mentioned before). But convex custom meshes, e.g. a pyramid block, are easily overlooked when preparing a simulation. Also, the decomposition of a highly convex mesh must probably be done manually without the help from a plugin. The aforementioned [V-HACD](https://github.com/kmammou/v-hacd) plugin will probably detect that the original mesh is already convex and perform none to minimal adjustments, which do not help with the Mujoco Simulation.

## 10.3. Timesteps:
When creating a scene, you can decide a "temporal resolution" of the simulation. The parameter `dt` governs, what length of each time step of the simulation is. The default values is `0.001`, meaning that 1 sec of simulated time consists of 1000 simulation steps. This of course is a tradeoff between simulation accuracy and performance, as smaller timesteps will result in a more realistic simulation, but also invoke additional computational load.

### 10.3.1. mujoco-py: nsubsteps
mujoco-py offers additional optimization potential, which we *DO NOT* expose to our users at the moment, but want to document here nonetheless.

A Mujoco environment / scene is defined via an XML file, which we parse and load into a [`mujocopy.MjSim`](https://openai.github.io/mujoco-py/build/html/reference.html#mjsim-basic-simulation) object. When creating this object, one can set an optional parameter `nsubsteps`:
```python
sim = mujocopy.MjSim(xml_model, nsubsteps=1)
```

Remember: Mujoco is a C++ physics simulator, mujoco-py is a library containing python bindings. If you know that you need high simulation accuracy, but only perform calculations which can handle coarser timesteps, you could change this parameter. Then, every call of the python function `sim.step()` will perform `nsubsteps` number of simulation steps in C++ at a resolution of `dt`. For example:

```python
1to1_sim = MjSim(model, nsubsteps=1) # t= 0.000
1to1_sim.step() # t = 0.001
1to1_sim.step() # t = 0.002 ...

1to100_sim = MjSim(model, nsubsteps=100) # t = 0.000
1to100_sim.step() # t = 0.100
1to100_sim.step() # t = 0.200 ...
```

## 10.4. Mujoco Advanced Physics Settings
In Mujoco there are a couple of parameters which define the physical behavior of your object. We have created a ["best practices"](https://www.overleaf.com/read/cfwjsrsgyynr) document, which you can follow to get an intuition for the most important parameters.

Furthermore the [Mujoco Documentation](http://www.mujoco.org/book/) will give you a very detailed overview of computation models and parameters.

[Back to Overview](./)

## 10.5. Converting .urdf to .xml files
In case we have an object as .urdf file (PyBullet format) available and want to load it in Mujoco, we need to convert it into a .xml file
. The steps for doing such a conversion are explained in this section.
### 1. Add Mujoco tags
Given the .xml file we want to add Mujoco tags such as <meshdir, balanceinertia, discardvisual> on top of the file. For more details on
tags see [here](https://mujoco.readthedocs.io/en/latest/XMLreference.html#compiler). An example can be found below.
```xml
<robot name="example">
    <mujoco>
        <compiler meshdir="../mesh/example_dir/" balanceinertia="true"/>
    </mujoco>
    <link name="MP_BODY">
        ...
</robot>
```
### 2. Check the mesh file format
The conversion only works if the mesh files are in .stl file format. Luckily there are many free (online) converters available that can
 convert .dae or .obj files into .stl. For example, a converter can be found [here](https://anyconv.com/de/obj-in-stl-konverter/).
 
### 3. Conversion
Enter the path `~/mujoco/mujoco200/bin` first and run the following command:
```
$./compile /path/to/model.urdf /path/to/model.xml
```
Note that `model.xml` is created by executing the command and should therefore not exists prior to the conversion.

### 4. Test the Model (optional)
We can look at the converted model in the interactive Mujoco simulator by running:
```
mujoco200/bin$ ./simulate /path/to/model.xml
```