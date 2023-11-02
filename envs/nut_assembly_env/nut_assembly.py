import numpy as np

import envs.nut_assembly_env.objects.nuts as nuts
import envs.nut_assembly_env.objects.pegs as pegs
from alr_sim.controllers.Controller import ControllerBase
from alr_sim.core import Scene
from alr_sim.gyms.gym_env_wrapper import GymEnvWrapper
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.gyms.gym_controllers import GymJointVelController


class NutAssemblyEnv(GymEnvWrapper):
    def __init__(
        self,
        simulator,
        n_substeps: int = 1,
        max_steps_per_episode: int = 2e3,
        debug: bool = False,
        random_env: bool = True,
    ):

        sim_factory = SimRepository.get_factory(simulator)

        scene = sim_factory.create_scene()
        robot = sim_factory.create_robot(scene)
        controller = GymJointVelController()

        super().__init__(
            scene=scene,
            controller=controller,
            max_steps_per_episode=max_steps_per_episode,
            n_substeps=n_substeps,
            debug=debug,
        )

        self.scene = scene
        self.robot = robot
        self.scene.add_object(pegs.SquarePeg())
        self.scene.add_object(pegs.RoundPeg())
        self.scene.add_object(nuts.SquareNut())
        self.scene.add_object(nuts.RoundNut())

    def get_observation(self) -> np.ndarray:
        return self.robot_state()

    def get_reward(self):
        return 0

    def _check_early_termination(self) -> bool:
        return False

    def _reset_env(self):
        pass
