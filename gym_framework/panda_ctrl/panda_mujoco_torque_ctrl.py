import gym
import numpy as np

from gym_framework.panda_ctrl.panda_mujoco_base_ctrl import PandaBase


class PandaTorqueControl(PandaBase):
    """
    Control the Panda robot by directly applying torques (control=torque).
    """

    def __init__(self, render=True):
        super().__init__(render=render)
        self._action_dimension = 8  # 7 x actuator torque; 1 gripper width

    def apply_action(self, action):
        assert len(action) == self.action_dimension, ("Error, wrong action dimension. Expected: " +
                                                      str(self.action_dimension) + ". Got:" + str(len(action)))

        # action = self.bound_action(action).copy()
        # self.panda.set_gripper_width = action[7]
        # torques = action[:7]
        # self.panda.command = torques
        # self.panda.nextStep()

        action = self.bound_action(action).copy()
        gripper_ctrl = action[7]
        joint_action = action[:7]

        # Set the joint command for the simulation
        self.sim.data.ctrl[:] = np.concatenate((joint_action, [gripper_ctrl, gripper_ctrl]))

        # Apply gravity compensation
        self.sim.data.qfrc_applied[self.joint_indices] = self.sim.data.qfrc_bias[self.joint_indices]

        # Forward the simulation
        self.sim.step()

        # Render the scene
        if self.render and self.viewer is not None:
            self.viewer.render()

    @property
    def ctrl_name(self):
        return 'torque'

    @property
    def action_dimension(self):
        return self._action_dimension

    @property
    def action_space(self):
        # upper and lower bounds on the actions
        low = [-87, -87, -87, -87, -12, -12, -12, -50]
        high = [87, 87, 87, 87, 12, 12, 12, 50]

        action_space = gym.spaces.Box(low=np.array(low),
                                      high=np.array(high))

        return action_space

    @property
    def state(self):
        current_joint_position = [self.sim.data.get_joint_qpos(j_name) for j_name in self.joint_names]
        current_joint_velocity = [self.sim.data.get_joint_qvel(j_name) for j_name in self.joint_names]

        current_finger_position = [self.sim.data.get_joint_qpos(j_name) for j_name in self.gripper_names]
        current_finger_velocity = [self.sim.data.get_joint_qvel(j_name) for j_name in self.gripper_names]

        tcp_pos = self.sim.data.get_body_xpos('tcp').copy()
        tcp_quat = self.sim.data.get_body_xquat('tcp').copy()
        tcp_velp = self.sim.data.get_body_xvelp('tcp').copy()
        tcp_velr = self.sim.data.get_body_xvelr('tcp').copy()

        return np.concatenate([current_joint_position,
                               current_joint_velocity,
                               current_finger_position,
                               current_finger_velocity,
                               tcp_pos,
                               tcp_quat,
                               tcp_velp,
                               tcp_velr])
