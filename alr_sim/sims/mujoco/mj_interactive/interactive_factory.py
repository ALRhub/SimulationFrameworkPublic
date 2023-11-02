import alr_sim.sims.SimFactory as Sims
from alr_sim.core.Robots import RobotBase
from alr_sim.sims.mujoco.mj_interactive.ia_robots.ia_mujoco_robot import (
    InteractiveMujocoRobot,
)
from alr_sim.sims.mujoco.MujocoFactory import MujocoFactory


class InteractiveMujocoFactory(MujocoFactory):
    def create_robot(self, *args, **kwargs) -> RobotBase:
        return InteractiveMujocoRobot(*args, **kwargs)


Sims.SimRepository.register(InteractiveMujocoFactory(), "mj_interactive")
