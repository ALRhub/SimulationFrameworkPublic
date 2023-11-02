# YCB Objects
To use the YCB Objects in your simulation you first have to clone the SF-ObjectDataset to your PC: https://github.com/ALRhub/SF-ObjectDataset

Then you can create any of the YCB Objects using the YCBMujocoObject Interface

```python
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import YCBMujocoObject
from alr_sim.sims.SimFactory import SimRepository

...
ycb_base_folder = "/home/nic/Projects/SF-ObjectDataset/YCB"
clamp = YCBMujocoObject(ycb_base_folder=ycb_base_folder, object_id="051_large_clamp", object_name='clamp', pos=[0.4, 0, 0.1], quat=[0, 0, 0, 1])

object_list = [clamp]

# Setup the scene
sim_factory = SimRepository.get_factory("mj_beta")

scene = sim_factory.create_scene(object_list=object_list)
robot = sim_factory.create_robot(scene)
scene.start()
```


# Old Version (still works, but is not as convenient)
## YCB Objects Generation

Copy the files `scripts/object_weights.yaml` and `scripts/ycb_downloader_and_converter.py` to a folder where you want to
store the YCB Object files.

```shell
pip install obj2mjcf
cp scripts/object_weights.yaml ycb_downloader_and_converter.py <YCBFOLDER>
cd <YCBFOLDER>
python ycb_downloader_and_converter.py
```

This script will first download all YCB Obj Files then go through each object and apply a convex decomposition to the
obj files and create Mujoco and Pybullet specific XML and URDF files.
The convex decomposition might take around an hour depending on the power of the used Computer.
Currently, this script still puts absolute paths in the Mujoco XML Files, the files can therefore not be moved after
initial creation

## Creating a YCB Object
Right now we mostly have the YCB Generation setup for the `mj_beta`. The old `mujoco` does sadly not work with the convex decompositions of the YCB Objects. Full pybullet integration still also needs a bit of time.

Here the code to create an example apple object:

```python
# FOR PYBULLET
from alr_sim.sims.pybullet.pb_utils.pybullet_scene_object import PyBulletURDFObject

apple_obj = PyBulletURDFObject(
    urdf_name="apple.urdf",
    object_name="apple",
    position=[0, 0, 0],
    orientation=[0, 0, 0], # Currently as euler
    data_dir=f"{path_to_object_folders}/013_apple/textured/"
)

# FOR MUJOCO
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CustomMujocoObject

apple_obj = CustomMujocoObject(
    object_name="apple",
    pos=[0, 0, 0],
    quat=[1, 0, 0, 0],
    root="/",
    object_dir_path=f"{path_to_object_folders}/013_apple/textured/"
)
```

Then just add the `apple_obj` to the object_list on scene creation.