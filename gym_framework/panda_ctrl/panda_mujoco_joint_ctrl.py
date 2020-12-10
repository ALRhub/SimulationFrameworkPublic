import gym
import numpy as np

from gym_framework.panda_ctrl.panda_mujoco_base_ctrl import PandaBase
from gym_framework.utils.helper import change_domain


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

        # Normalize the gripper action from [-1, 1] to [0, 0.04]
        gripper_ctrl = change_domain(action[-1], in_low=-1, in_high=1, out_low=0, out_high=0.04)
        joint_action = action[:7]  # todo check if actions have to be normalized

        # Set the joint command for the simulation
        self.sim.data.ctrl[:] = np.concatenate(([gripper_ctrl, gripper_ctrl], joint_action))

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
        dt = self.sim.nsubsteps * self.sim.model.opt.timestep
        finger1_pos = [self.sim.data.get_joint_qpos('panda_finger_joint1').copy()]
        finger2_pos = [self.sim.data.get_joint_qpos('panda_finger_joint2').copy()]

        # Transform gripper observation to [-1, 1]
        finger1_pos = [change_domain(finger1_pos[0], in_low=0, in_high=0.04, out_low=-1, out_high=1)]
        finger2_pos = [change_domain(finger2_pos[0], in_low=0, in_high=0.04, out_low=-1, out_high=1)]

        current_joint_position = [self.sim.data.get_joint_qpos(j_name) for j_name in self.joint_names]
        current_joint_velocity = [self.sim.data.get_joint_qvel(j_name) * dt for j_name in self.joint_names]

        tcp_pos = self.sim.data.get_body_xpos('tcp').copy()
        tcp_quat = self.sim.data.get_body_xquat('tcp').copy()
        tcp_velp = self.sim.data.get_body_xvelp('tcp').copy() * dt
        tcp_velr = self.sim.data.get_body_xvelr('tcp').copy() * dt

        return np.concatenate([finger1_pos,
                               finger2_pos,
                               current_joint_position,
                               current_joint_velocity,
                               tcp_pos,
                               tcp_quat,
                               tcp_velp,
                               tcp_velr])


class PandaJointVelControl(PandaJointControl):
    def __init__(self, render=True):
        super().__init__(render=render)

    @property
    def action_space(self):
        # upper and lower bound for each joint velocities given by
        # https://frankaemika.github.io/docs/control_parameters.html#controller-requirements
        low = [-2.175, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100, -1]
        high = [2.175, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 1]

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
        # upper and lower bound for each joint position given by
        # https://frankaemika.github.io/docs/control_parameters.html#controller-requirements
        low = [-2.8973, - 1.7628, - 2.8973, - 3.0718, - 2.8973, - 0.0175, - 2.8973, -1]
        high = [2.8973, 1.7628, 2.8973, - 0.0698, 2.8973, 3.7525, 2.8973, 1]

        action_space = gym.spaces.Box(low=np.array(low),
                                      high=np.array(high))

        return action_space

    @property
    def ctrl_name(self):
        return 'position'
