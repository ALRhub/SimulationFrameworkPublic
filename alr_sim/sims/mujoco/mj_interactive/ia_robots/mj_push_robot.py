from alr_sim.sims.mujoco.MujocoRobot import MujocoRobot
from alr_sim.utils.sim_path import sim_framework_path


class MjPushRobot(MujocoRobot):
    @property
    def xml_file_path(self):
        return sim_framework_path("./models/mujoco/robots/panda_rod.xml")
