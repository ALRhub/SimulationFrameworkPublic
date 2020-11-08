import numpy as np
from unittest import TestCase
from gym_framework.mujoco_envs.pick_and_place_env.pick_and_place_env import PickAndPlaceMocapCtrl
from gym_framework.utils.helper import has_collision


def rndmGripperAction():
    mean = -1
    std = 0.0
    eps = np.random.randn()
    return np.tanh(mean + eps * std)


class TestReachEnvMocapCtrl(TestCase):
    def setUp(self) -> None:
        self.env = PickAndPlaceMocapCtrl(render=1, max_steps=12000, nsubsteps=3, random_env=False)
        self.sim = self.env.sim

    def testRndActions(self):
        for i in range(10000):
            action = self.env.action_space.sample()
            state, reward, done, _ = self.env.step(action)
            if done:
                self.env.reset()

    def testPickAndPlace(self):
        des_pos = self.sim.data.get_body_xpos('box').copy()
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            obs, r, done, _ = self.env.step(np.concatenate([action, [1]]))

        for _ in range(2000):
            action = np.array([0, 0, 0])
            obs, r, done, _ = self.env.step(np.concatenate([action, [-1]]))

        des_pos = self.sim.data.get_body_xpos('goal').copy()
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            obs, r, done, _ = self.env.step(np.concatenate([action, [-1]]))

        while True:
            action = np.array([0, 0, 0])
            obs, r, done, _ = self.env.step(np.concatenate([action, [-1]]))

    def testPickRandomAction(self):
        np.random.seed(123)
        des_pos = self.sim.data.get_body_xpos('box').copy()
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        for _ in range(50):
            action = np.array([0, 0, 0])
            self.env.step(np.concatenate([action, [-1]]))

        action = np.array([0, 0, 1])
        for _ in range(400):
            self.env.step(np.concatenate([action[:3], [-1]]))
        action = np.array([0, 0, 0.0])
        for _ in range(200):
            self.env.step(np.concatenate([action[:3], [-1]]))

        action = np.array([0, 0.5, 0.0])
        for _ in range(20):
            self.env.step(np.concatenate([action[:3], [-1]]))
        action = np.array([0, 0, 0.0])
        for _ in range(500):
            self.env.step(np.concatenate([action[:3], [-1]]))

        for _ in range(5000):
            action = self.env.action_space.sample()
            gripper_act = -1  # rndmGripperAction()
            self.env.step(np.concatenate([action[:3], [gripper_act]]))

    def testPickAndPlaceRepeat(self):
        for _ in range(10):
            des_pos = self.sim.data.get_body_xpos('box').copy()
            while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
                action = des_pos - self.sim.data.get_body_xpos('tcp')
                self.env.step(np.concatenate([action, [1]]))

            for __ in range(300):
                action = np.array([0, 0, 0])
                self.env.step(np.concatenate([action, [-1]]))

            des_pos = self.sim.data.get_body_xpos('goal').copy()
            while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
                action = des_pos - self.sim.data.get_body_xpos('tcp')
                self.env.step(np.concatenate([action, [-1]]))

            for __ in range(300):
                action = np.array([0, 0, 0])
                self.env.step(np.concatenate([action, [0.04]]))

    def testWorkspace(self):
        """ Tests whether the gripper can reach the table top. It should not work since the z-axis is constrained such
        that the robot can not touch the table top.
        """
        init_pos = self.sim.data.get_body_xpos('tcp').copy()
        for _ in range(500):
            action = [0, 0, 0]
            self.env.step(np.concatenate([action, [1]]))

        des_pos = np.array([0.5, 0, -0.5])  # Arbitrary position below workspace constrains
        for _ in range(500):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-1):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            print(des_pos, self.sim.data.get_body_xpos('tcp'))
            self.env.step(np.concatenate([action, [1]]))

        des_pos = np.array([0.5, 0, 1])  # Arbitrary position above workspace constrains
        for _ in range(500):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos.copy()
        des_pos[0] += 1  # Arbitrary position in front workspace constrains
        for _ in range(500):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos.copy()
        des_pos[0] -= 1  # Arbitrary position behind workspace constrains
        for _ in range(500):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos.copy()
        des_pos[1] -= 1  # Arbitrary position left of workspace constrains
        for _ in range(500):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos.copy()
        des_pos[1] += 1  # Arbitrary position right of workspace constrains
        for _ in range(500):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        des_pos = init_pos
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

    def testTerminationCond(self):
        des_pos = self.sim.data.get_body_xpos('box').copy()
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [1]]))

        for _ in range(500):
            action = np.array([0, 0, 0])
            self.env.step(np.concatenate([action, [-1]]))

        des_pos = self.sim.data.get_site_xpos('goal:site1').copy()
        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-4):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [-1]]))

        self.assertTrue(self.env._termination() is True)

    def testVelocity(self):
        for _ in range(1000):
            action = np.array([1, 1, 1])
            self.env.step(np.concatenate([action, [1]]))

    def testObservationBounds(self):
        for i in range(10000):
            action = self.env.action_space.sample()
            state, reward, done, _ = self.env.step(action)
            self.assertTrue((-1 <= state).all())
            self.assertTrue((state <= 1).all())
            if done:
                self.env.reset()

    def testPushBox(self):
        des_pos = self.sim.data.get_body_xpos('box').copy()
        des_pos[2] -= 0.3

        for _ in range(500):
            action = np.array([0, 0, 0])
            self.env.step(np.concatenate([action, [-1]]))

        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [-1]]))

    def testPushBoxSide(self):
        des_pos = self.sim.data.get_body_xpos('box').copy()
        des_pos[2] -= 0.3
        des_pos[1] -= 0.01

        for _ in range(500):
            action = np.array([0, 0, 0])
            self.env.step(np.concatenate([action, [-1]]))

        while not np.allclose(self.sim.data.get_body_xpos('tcp'), des_pos, atol=1e-2):
            action = des_pos - self.sim.data.get_body_xpos('tcp')
            self.env.step(np.concatenate([action, [-1]]))
