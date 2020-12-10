import numpy as np
from unittest import TestCase
from gym_framework.mujoco_envs.reach_env.reach_env import ReachEnvTorqueCtrl


class TestReachEnvTorqueCtrl(TestCase):
    def setUp(self) -> None:
        self.env = ReachEnvTorqueCtrl(nsubsteps=12, render=True)

    def testRndActions(self):
        for i in range(1000):
            action = self.env.action_space.sample()
            # action = np.zeros([self.env.action_dim])
            obs, reward, done, _ = self.env.step(action)
            if done:
                self.env.reset()