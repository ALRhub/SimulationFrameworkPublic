import gym
import mujoco_py
import numpy as np

from gym_framework.panda_ctrl.panda_mujoco_base_ctrl import PandaBase
from gym_framework.utils.helper import change_domain


class PandaMocapControl(PandaBase):
    """
    The action controls the robot using mocaps. Specifically, bodies
    on the robot (for example the gripper wrist) is controlled with
    mocap bodies. In this case the action is the desired difference
    in position and orientation (quaternion), in world coordinates,
    of the of the target body. The mocap is positioned relative to
    the target body according to the delta, and the MuJoCo equality
    constraint optimizer tries to center the welded body on the mocap.
    """

    def __init__(self,
                 render=True,
                 fixed_orientation=True,
                 orientation_delta_limit=0.1,
                 coordinates='relative',
                 percentage=0.005):

        super().__init__(render=render)

        self.fixed_orientation = fixed_orientation
        self.orientation_delta_limit = orientation_delta_limit
        self.percentage = percentage
        self.coordinates = coordinates
        if self.fixed_orientation:
            self._action_dimension = 4  # 3 x direction, 1 x gripper width
        else:
            self._action_dimension = 8  # 3 x direction, 4 x orientation,  1 x gripper width

    @property
    def ctrl_name(self):
        return 'mocap'

    def env_setup(self, sim, viewer):
        """Resets the mocap welds that are used actuation.
        """
        super().env_setup(sim, viewer)
        self.workspace = self.get_workspace()
        if self.sim.model.nmocap > 0 and self.sim.model.eq_data is not None:
            for i in range(self.sim.model.eq_data.shape[0]):
                if self.sim.model.eq_type[i] == mujoco_py.const.EQ_WELD:
                    self.sim.model.eq_data[i, :] = np.array(
                        [0., 0., 0., 1., 0., 0., 0.])

        self.sim.forward()

        # Move end effector into position.
        gripper_target = self.sim.data.get_body_xpos('tcp')
        gripper_rotation = np.array([0., 1., 0., 0.])
        self.sim.data.set_mocap_pos('panda:mocap', gripper_target)
        self.sim.data.set_mocap_quat('panda:mocap', gripper_rotation)
        for _ in range(10):
            self.sim.step()

    def get_workspace(self):
        workspace_low = np.array([self.sim.data.get_site_xpos('x_constrain_low')[0],
                                  self.sim.data.get_site_xpos('y_constrain_low')[1],
                                  self.sim.data.get_site_xpos('z_constrain_low')[2]])

        workspace_high = np.array([self.sim.data.get_site_xpos('x_constrain_high')[0],
                                   self.sim.data.get_site_xpos('y_constrain_high')[1],
                                   self.sim.data.get_site_xpos('z_constrain_high')[2]])

        return gym.spaces.Box(low=workspace_low, high=workspace_high)

    @property
    def action_dimension(self):
        return self._action_dimension

    @property
    def action_space(self):
        # upper and lower bound on the actions
        if self.coordinates == 'relative':
            low_x, low_y, low_z, low_gripper = -1, -1, -1, -1
            high_x, high_y, high_z, high_gripper = 1, 1, 1, 1
        elif self.coordinates == 'absolute':
            low_x, low_y, low_z, low_gripper = -0.3, -0.5, 0., 0
            high_x, high_y, high_z, high_gripper = 0.7, 0.5, 0.7, 0.04
        else:
            raise ValueError("Error, choose between <absolute> (the given a action is a coordinate in 3D space) or "
                             "<relative> (the given action is a directional vector).")

        quat_w_low, quat_x_low, quat_y_low, quat_z_low = [-1, -1, -1, -1]
        quat_w_high, quat_x_high, quat_y_high, quat_z_high = [1, 1, 1, 1]

        if self.fixed_orientation:
            action_space = gym.spaces.Box(low=np.array([low_x, low_y, low_z, low_gripper]),
                                          high=np.array([high_x, high_y, high_z, high_gripper]))
        else:
            action_space = gym.spaces.Box(low=np.array([low_x, low_y, low_z,
                                                        quat_w_low, quat_x_low, quat_y_low, quat_z_low]),
                                          high=np.array([high_x, high_y, high_z,
                                                         quat_w_high, quat_x_high, quat_y_high, quat_z_high]))

        return action_space

    @property
    def state(self):
        # todo add gripper force?
        dt = self.sim.nsubsteps * self.sim.model.opt.timestep
        finger1_pos = [self.sim.data.get_joint_qpos('panda_finger_joint1').copy()]
        finger2_pos = [self.sim.data.get_joint_qpos('panda_finger_joint2').copy()]

        # Transform gripper observation to [-1, 1]
        finger1_pos = [change_domain(finger1_pos[0], in_low=0, in_high=0.04, out_low=-1, out_high=1)]
        finger2_pos = [change_domain(finger2_pos[0], in_low=0, in_high=0.04, out_low=-1, out_high=1)]

        tcp_pos = self.sim.data.get_body_xpos('tcp').copy()
        tcp_velp = self.sim.data.get_body_xvelp('tcp').copy() * dt
        if self.fixed_orientation:
            return np.concatenate([finger1_pos, finger2_pos, tcp_pos, tcp_velp])
        else:
            tcp_quat = self.sim.data.get_body_xquat('tcp').copy()
            tcp_velr = self.sim.data.get_body_xvelr('tcp').copy()
            return np.concatenate([finger1_pos, finger2_pos, tcp_pos, tcp_quat, tcp_velp, tcp_velr])

    @property
    def state_bounds(self):
        finger1_low = finger2_low = [0]
        finger1_high = finger2_high = [0.04]
        tcp_pos_low = self.workspace.low
        tcp_pos_high = self.workspace.high
        tcp_velp_low = [-self.percentage, -self.percentage, -self.percentage]  # Todo
        tcp_velp_high = [self.percentage, self.percentage, self.percentage]  # Todo
        if self.fixed_orientation:
            return np.concatenate([finger1_low, finger2_low, tcp_pos_low, tcp_velp_low]), \
                   np.concatenate([finger1_high, finger2_high, tcp_pos_high, tcp_velp_high])
        else:
            tcp_quat_low = [0, 0, 0, 0]
            tcp_quat_high = [1, 1, 1, 1]
            tcp_velr_low = ...
            tcp_velr_high = ...
            return np.concatenate([finger1_low, finger2_low, tcp_pos_low, tcp_quat_low, tcp_velp_low, tcp_velr_low]), \
                   np.concatenate([finger1_high, finger2_high, tcp_pos_high, tcp_quat_high, tcp_velp_high,
                                   tcp_velr_high])

    def apply_action(self, action):
        assert len(action) == self._action_dimension, ("Error, wrong action dimension. Expected dim = 4 if fixed "
                                                       "orientation and 8 otherwise. Got " + str(len(action)))

        # Normalize the gripper action from [-1, 1] to [0, 0.04]
        action[-1] = change_domain(action[-1], in_low=-1, in_high=1, out_low=0, out_high=0.04)

        if self.fixed_orientation:
            # pos_ctrl, orientation, gripper_width = action[:3], [0., 1., 0., 0.], action[3]
            pos_ctrl, orientation = action[:3], [0., 1., 0., 0.]
        else:
            pos_ctrl, orientation, gripper_width = action[:3], action[3:7], action[7]

        self.sim.data.qfrc_applied[self.joint_indices] = self.sim.data.qfrc_bias[self.joint_indices]

        # Set the position control
        if self.coordinates == 'absolute':
            direction = pos_ctrl - self.sim.data.mocap_pos
        elif self.coordinates == 'relative':
            direction = np.array(pos_ctrl)
        else:
            raise ValueError("Error, choose between <absolute> (the given a action is a coordinate in 3D space) or "
                             "<relative> (the given action is a directional vector).")

        # Limit the maximum length of the directional vector the robot should move
        position_delta = direction * self.percentage

        self.reset_mocap2body_xpos()
        self.sim.data.ctrl[:] = action[3]
        desired_pos = self.sim.data.mocap_pos + position_delta
        # Constrains the workspace
        desired_pos = np.clip(desired_pos, a_min=self.workspace.low, a_max=self.workspace.high)

        self.sim.data.mocap_pos[:] = desired_pos
        if self.fixed_orientation:
            self.sim.data.mocap_quat[:] = orientation
        else:
            self.sim.data.mocap_quat[:] = self.sim.data.mocap_quat + self.orientation_delta_limit * orientation

        # Run the simulation
        self.sim.step()

        # Render the scene
        if self.render and self.viewer is not None:
            self.viewer.render()

    def reset_mocap2body_xpos(self):
        """Resets the position and orientation of the mocap bodies to the same
        values as the bodies they're welded to.
        """
        if (self.sim.model.eq_type is None or
                self.sim.model.eq_obj1id is None or
                self.sim.model.eq_obj2id is None):
            return
        for eq_type, obj1_id, obj2_id in zip(self.sim.model.eq_type,
                                             self.sim.model.eq_obj1id,
                                             self.sim.model.eq_obj2id):
            if eq_type != mujoco_py.const.EQ_WELD:
                continue

            mocap_id = self.sim.model.body_mocapid[obj1_id]
            if mocap_id != -1:
                # obj1 is the mocap, obj2 is the welded body
                body_idx = obj2_id
            else:
                # obj2 is the mocap, obj1 is the welded body
                mocap_id = self.sim.model.body_mocapid[obj2_id]
                body_idx = obj1_id

            assert (mocap_id != -1)
            self.sim.data.mocap_pos[mocap_id][:] = self.sim.data.body_xpos[body_idx]
            self.sim.data.mocap_quat[mocap_id][:] = self.sim.data.body_xquat[body_idx]
