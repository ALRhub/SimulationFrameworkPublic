import numpy as np
from unittest import TestCase
import matplotlib.pyplot as plt
from gym_framework.mujoco_envs.pick_and_place_env.pick_and_place_env import PickAndPlaceMocapCtrl


def rndmGripperAction():
    mean = -1
    std = 0.7
    eps = np.random.randn()
    return np.tanh(mean + eps * std)


def normalize(value, out_range=None, in_range=None):
    """ Normalized the observations which are in [observation_space.low, observation_space.high] to [lower, upper].
    """
    if in_range is None:
        in_range = [0, 0.04]
    if out_range is None:
        out_range = [-1, 1]

    return (out_range[1] - out_range[0]) * (
            (value - in_range[0]) / (in_range[1] - in_range[0])) + out_range[0]


class TestReachEnvMocapCtrl(TestCase):
    def setUp(self) -> None:
        self.env = PickAndPlaceMocapCtrl(render=0, max_steps=2000, nsubsteps=3, random_env=False)
        self.sim = self.env.sim

    def testRndActions(self):
        for i in range(10000):
            action = self.env.action_space.sample()
            state, reward, done, _ = self.env.step(action)
            if done:
                self.env.reset()

    def testGrasping(self):
        for ntimesteps in [5, 20, 50, 100, 500]:
            des_gripper_pos = np.linspace(1, -1, num=ntimesteps)
            act_gripper_pos = []
            for gripper_pos in des_gripper_pos:
                action = [0, 0, 0]
                obs, r, done, _ = self.env.step(np.concatenate([action, [gripper_pos]]))
                print(self.env.sim.data.get_joint_qpos('panda_finger_joint1').copy())
                act_gripper_pos.append(normalize(self.env.sim.data.get_joint_qpos('panda_finger_joint1').copy()))

            for _ in range(500):
                action = [0, 0, 0]
                self.env.step(np.concatenate([action, [1]]))

            plt.plot(des_gripper_pos)
            plt.plot(act_gripper_pos)
            plt.show()

    def testGraspingRndm(self):
        des_gripper_pos = []
        act_gripper_pos = []
        for _ in range(1000):
            action = [0, 0, 0]
            gripper_act = rndmGripperAction()
            des_gripper_pos.append(gripper_act)
            obs, r, done, _ = self.env.step(np.concatenate([action, [gripper_act]]))
            print(self.env.sim.data.get_joint_qpos('panda_finger_joint1').copy())
            act_gripper_pos.append(normalize(self.env.sim.data.get_joint_qpos('panda_finger_joint1').copy()))

        for _ in range(500):
            action = [0, 0, 0]
            self.env.step(np.concatenate([action, [1]]))

        plt.plot(des_gripper_pos)
        plt.plot(act_gripper_pos)
        plt.show()

    def testGripperMaxVel(self):
        """ Tests if the actions are correctly given to the environment.
        """
        minVel = np.inf
        maxVel = -np.inf
        for _ in range(500):
            action = np.array([0, 0, 0])
            self.env.step(np.concatenate([action, [-1]]))
            # print(self.env.sim.data.get_joint_qvel("panda_finger_joint2"))
            minVel = min(minVel, self.env.sim.data.get_joint_qvel("panda_finger_joint2"))
            maxVel = max(maxVel, self.env.sim.data.get_joint_qvel("panda_finger_joint2"))

        for _ in range(500):
            action = np.array([0, 0, 0])
            self.env.step(np.concatenate([action, [1]]))
            # print(self.env.sim.data.get_joint_qvel("panda_finger_joint2"))
            minVel = min(minVel, self.env.sim.data.get_joint_qvel("panda_finger_joint2"))
            maxVel = max(maxVel, self.env.sim.data.get_joint_qvel("panda_finger_joint2"))

        for _ in range(500):
            action = np.array([0, 0, 0])
            self.env.step(np.concatenate([action, [-1]]))

        print(minVel, maxVel)
