from abc import abstractmethod

import numpy as np
from classic_framework.mujoco.mujoco_utils.mujoco_controllers import \
    MujocoController
from classic_framework.utils.sim_path import sim_framework_path


class PandaBase(MujocoController):
    """
    Base class for different control settings of the panda.
    """

    def __init__(self, render=True, num_dof=7):
        self.sim = None
        self.viewer = None
        self.joint_names = None
        self.gripper_names = None
        self.joint_indices = [x for x in range(1, num_dof + 1)]
        self.render = render
        self.num_dof = num_dof

    def env_setup(self, sim, viewer):
        self.sim = sim
        self.viewer = viewer

        self.joint_names = [
            name for name in self.sim.model.joint_names if name.startswith('panda_joint')]
        assert len(
            self.joint_names) == self.num_dof, "Error, found more joints than expected."

        self.gripper_names = [
            name for name in self.sim.model.joint_names if name.startswith('panda_finger_joint')]
        assert len(
            self.gripper_names) == 2, "Error, found more gripper joints than expected."

    def bound_action(self, action):
        """Bounds the action to the [-1 1]. WARNING: This should not have an effect since the actions from the policy
        should already be in [-1,1]. This works as a backup.
        """
        return np.clip(action, a_min=-1, a_max=1)

    def normalize_action(self, action, lower=-1, upper=1):
        """Normalizes the action to the the given range [lower, upper].
        """
        return (self.action_space.high - self.action_space.low) * (
            (action - lower) / (upper - lower)) + self.action_space.low

    def preprocess_action(self, action):
        """Bounds actions coming from the policy to [-1, 1] and normalizes them to [action_space.low, action_space.high]
        """
        return self.normalize_action(self.bound_action(action))

    @property
    def tcp_pos(self):
        """Getter for the tcp (tool center point or end effector).
        """
        return self.sim.data.get_body_xpos('tcp').copy()

    @property
    @abstractmethod
    def state(self):
        """Getter for the state of the robot. The state depends on the type of robot controlling.
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def action_space(self):
        """
        Returns the action space with upper and lower bounds for the respective actions.

        Returns:
            gym_envs.spaces.Box; action space
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def action_dimension(self) -> int:
        """Returns the dimension of the action space. Implement this in all subclasses since the action dimension varies
        with different controlling of the robot.
        """
        raise NotImplementedError

    @abstractmethod
    def apply_action(self, action) -> None:
        """Applies the given action and advances the simulation. Each control needs a different implementation of this
        method.
        """
        raise NotImplementedError
