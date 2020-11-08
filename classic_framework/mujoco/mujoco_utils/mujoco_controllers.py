import abc
import logging

from classic_framework.mujoco.mujoco_utils.mujoco_scene_object import MujocoXmlLoadable
from classic_framework.utils.sim_path import sim_framework_path


class MujocoController(MujocoXmlLoadable):

    @property
    @abc.abstractmethod
    def ctrl_name(self):
        pass

    @property
    def gripper_ctrl_name(self):
        return 'ik'

    @property
    def _xml_name(self):
        return self.ctrl_name

    @property
    def xml_file_path(self):
        return sim_framework_path('./envs/mujoco/panda/controller/panda_{}_control.xml'.format(self._xml_name))


class IKControl(MujocoController):
    @property
    def ctrl_name(self):
        return 'ik'

    @property
    def _xml_name(self):
        return 'torque'


class TorqueControl(MujocoController):
    @property
    def ctrl_name(self):
        return 'torque'


class PositionControl(MujocoController):
    @property
    def ctrl_name(self):
        return 'position'


class VelocityControl(MujocoController):
    @property
    def ctrl_name(self):
        return 'velocity'


class MocapControl(MujocoController):
    @property
    def ctrl_name(self):
        return 'mocap'


def convert_str2control(name: str) -> MujocoController:
    # TODO: Remove in Future!
    logging.warning(
        "Please use mujoco_controllers.MujocoControl objects. Setting a controller by name will be deprecated in the future.")
    if name == 'ik':
        return IKControl()
    if name == 'torque':
        return TorqueControl()
    if name == 'position':
        return PositionControl()
    if name == 'velocity':
        return VelocityControl()
    if name == 'mocap':
        return MocapControl()
