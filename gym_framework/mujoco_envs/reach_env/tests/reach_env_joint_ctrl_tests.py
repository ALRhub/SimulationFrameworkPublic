import numpy as np
from unittest import TestCase
from gym_framework.mujoco_envs.reach_env.reach_env import ReachEnvJointVelCtrl, ReachEnvJointPosCtrl


class TestReachEnvJointVelCtrl(TestCase):
    def setUp(self) -> None:
        self.env = ReachEnvJointVelCtrl(nsubsteps=20, render=True)

    def testRndActions(self):
        for i in range(1000):
            action = self.env.action_space.sample()
            obs, reward, done, _ = self.env.step(action)
            if done:
                self.env.reset()


class TestReachEnvJointPosCtrl(TestCase):
    def setUp(self) -> None:
        self.env = ReachEnvJointPosCtrl(nsubsteps=10, render=True)

    def testRndActions(self):
        for i in range(1000):
            action = self.env.action_space.sample()
            obs, reward, done, _ = self.env.step(action)
            if done:
                self.env.reset()
