import gym
import numpy as np

from gym_framework.panda_ctrl.panda_mujoco_base_ctrl import PandaBase


class PandaJointControl(PandaBase):
    """
    Control the panda by specifying joint positions (control=position) / velocities (control=velocity).
    """

    def __init__(self,
                 render=True):
        super().__init__(render=render)

        self._action_dimension = 8  # 7 x actuator joint; 1 gripper width

    def apply_action(self, action):
        assert len(action) == self.action_dimension, ("Error, wrong action dimension. Expected: " +
                                                      str(self.action_dimension) + ". Got:" + str(len(action)))

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
    def action_dimension(self):
        return self._action_dimension

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


class PandaJointVelControl(PandaJointControl):
    def __init__(self, render=True):
        super().__init__(render=render)

    @property
    def action_space(self):
        # upper and lower bound for each joint velocity
        low = [-2.175, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.9671, -1]
        high = [2.175, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.9671, 1]

        action_space = gym.spaces.Box(low=np.array(low),
                                      high=np.array(high))

        return action_space

    @property
    def ctrl_name(self):
        return 'velocity'


class PandaJointPosControl(PandaJointControl):
    def __init__(self, render=True):
        super().__init__(render=render)

    @property
    def action_space(self):
        # upper and lower bound for each joint position
        low = [-2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -3.7525, -2.9671, -1]
        high = [2.9671, 1.8326, 2.9671, 0.0, 2.9671, 2.1817, 2.9671, 1]

        action_space = gym.spaces.Box(low=np.array(low),
                                      high=np.array(high))

        return action_space

    @property
    def ctrl_name(self):
        return 'position'
