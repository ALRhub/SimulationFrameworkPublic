from typing import List

# from pyrosenv import rospy
import numpy as np

from alr_sim.core.Scene import Scene
from alr_sim.core.sim_object.sim_object import IntelligentSimObject, SimObject
from alr_sim.sims.sl_ros.RosLocalizer import RosLocalizer


class RosScene(Scene):
    def __init__(
        self,
        object_list=None,
        dt=0.001,
        render=Scene.RenderMode.HUMAN,
        cam_list=[],
        calib_path="",
        calib_cams=[],
    ):
        super(RosScene, self).__init__(object_list=object_list, dt=dt, render=render)
        self.cam_list = cam_list
        self._localizer = RosLocalizer(calib_cams, calib_path)

    @property
    def sim_name(self) -> str:
        return "sl_ros"

    def _get_obj_pos(self, poi, sim_obj: SimObject):
        return self._localizer.locate_pos(sim_obj, self.cam_list)

    def _get_obj_quat(self, poi, sim_obj: SimObject):
        return self._localizer.locate_quat(sim_obj, self.cam_list)

    def _get_obj_pos_and_quat(self, poi, sim_obj: SimObject):
        return self._localizer.locate_pos_and_quat(sim_obj, self.cam_list)

    def position_updated(self):
        return self._localizer.position_updated()

    def _remove_object(self, sim_obj: SimObject):
        return None

    def _rt_add_object(self, sim_obj: SimObject):
        return None

    def _set_obj_pos(self, new_pos, sim_obj: SimObject):
        return None

    def _set_obj_quat(self, new_quat, sim_obj: SimObject) -> np.ndarray:
        return None

    def _set_obj_pos_and_quat(
        self, new_pos, new_quat, sim_obj: SimObject
    ) -> np.ndarray:
        return None

    def _setup_objects(self, sim_objs: List[SimObject]):
        return None

    def _setup_scene(self):
        return None

    def _sim_step(self):
        return None

    def load_robot_to_scene(self, robot_init_qpos: np.ndarray = None):
        return None

    def render(self):
        return None

    def reset(self, obj_pos=None):
        return None
