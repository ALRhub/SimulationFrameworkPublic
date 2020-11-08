import numpy as np
from unittest import TestCase
from gym_framework.mujoco_envs.reach_env.reach_env import ReachEnvMocapCtrl


class TestReachEnvMocapCtrl(TestCase):
    def setUp(self) -> None:
        self.env = ReachEnvMocapCtrl(render=True)

    def testRndActions(self):
        for i in range(1000):
            action = self.env.action_space.sample()
            obs, reward, done, _ = self.env.step(action)
            if done:
                self.env.reset()
