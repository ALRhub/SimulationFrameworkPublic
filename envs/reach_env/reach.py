import numpy as np
from gym.spaces import Box as SamplingSpace

from alr_sim.controllers.Controller import ControllerBase
from alr_sim.core import Scene
from alr_sim.gyms.gym_env_wrapper import GymEnvWrapper
from alr_sim.gyms.gym_utils.helpers import obj_distance
from alr_sim.sims.universal_sim.PrimitiveObjects import Sphere


class ReachEnv(GymEnvWrapper):
    def __init__(
        self,
        scene: Scene,
        controller: ControllerBase,
        n_substeps: int = 25,
        max_steps_per_episode: int = 2e3,
        debug: bool = False,
        random_env: bool = False,
    ):
        super().__init__(
            scene=scene,
            controller=controller,
            max_steps_per_episode=max_steps_per_episode,
            n_substeps=n_substeps,
            debug=debug,
        )

        self.random_env = random_env

        self.goal = Sphere(
            name="goal",
            size=[0.01],
            init_pos=[0.5, 0, 0.1],
            init_quat=[1, 0, 0, 0],
            rgba=[1, 0, 0, 1],
            static=True,
        )
        self.goal_space = SamplingSpace(
            low=np.array([0.2, -0.3, 0.1]), high=np.array([0.5, 0.3, 0.5])
        )
        self.scene.add_object(self.goal)

        self.target_min_dist = 0.02

    def get_observation(self) -> np.ndarray:
        goal_pos = self.scene.get_obj_pos(self.goal)
        tcp_pos = self.robot.current_c_pos
        dist_tcp_goal, rel_goal_tcp_pos = obj_distance(goal_pos, tcp_pos)

        env_state = np.concatenate([goal_pos, [dist_tcp_goal], rel_goal_tcp_pos])
        robot_state = self.robot_state()
        return np.concatenate([robot_state, env_state])

    def get_reward(self):
        goal_pos = self.scene.get_obj_pos(self.goal)
        tcp_pos = self.robot.current_c_pos
        dist_tcp_goal, _ = obj_distance(goal_pos, tcp_pos)
        reward = np.double(-dist_tcp_goal)

        if dist_tcp_goal <= self.target_min_dist:
            reward = np.double(10.0)
        return reward

    def _check_early_termination(self) -> bool:
        # calculate the distance from end effector to object
        goal_pos = self.scene.get_obj_pos(self.goal)
        tcp_pos = self.robot.current_c_pos
        dist_tcp_goal, _ = obj_distance(goal_pos, tcp_pos)

        if dist_tcp_goal <= self.target_min_dist:
            # terminate if end effector is close enough
            self.terminated = True
            return True
        return False

    def _reset_env(self):
        if self.random_env:
            new_goal = [self.goal, self.goal_space.sample()]
            self.scene.reset([new_goal])
        else:
            self.scene.reset()
