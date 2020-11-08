from classic_framework.mujoco.mujoco_utils import mujoco_scene_object
from classic_framework.utils.sim_path import sim_framework_path


class Box(mujoco_scene_object.MujocoXmlLoadable):
    ID = 'box'

    @property
    def xml_file_path(self):
        return sim_framework_path('gym_framework', 'mujoco_objects', 'assets', 'box.xml')


class Goal(mujoco_scene_object.MujocoXmlLoadable):
    ID = 'goal'

    @property
    def xml_file_path(self):
        return sim_framework_path('gym_framework', 'mujoco_objects', 'assets', 'goal.xml')
