import numpy as np

from gym_framework.mujoco_envs.mujoco_env import MujocoEnv
from gym_framework.mujoco_objects import env_objects
from gym_framework.panda_ctrl.panda_mujoco_joint_ctrl import PandaJointPosControl
from gym_framework.panda_ctrl.panda_mujoco_joint_ctrl import PandaJointVelControl
from gym_framework.panda_ctrl.panda_mujoco_mocap_ctrl import PandaMocapControl
from gym_framework.panda_ctrl.panda_mujoco_torque_ctrl import PandaTorqueControl
from gym_framework.utils.helper import obj_distance


class ReachEnvBase(MujocoEnv):
    """
    Reach task: The agent is asked to go to a certain object and gets reward when getting closer to the object. Once
    the object is reached, the agent gets a high reward.
    """

    def __init__(self, agent, render=True, max_steps=2000,
                 nsubsteps=1, dt=2e-3, random_env=True, workspace_size='medium'):
        self._box = env_objects.Box()
        super().__init__(agent=agent, obj_list=[self._box], max_steps=max_steps, render=render,
                         nsubsteps=nsubsteps, dt=dt, random_env=random_env, workspace_size=workspace_size)
        self.target_min_dist = 0.02

    @property
    def environment_observations(self):
        box_pos = self.sim.data.get_body_xpos(self._box.ID).copy()
        dist_tcp_box, _ = obj_distance(self.sim, self._tcp_id, self._box.ID)
        rel_box_tcp_pos = box_pos - self.sim.data.get_body_xpos('tcp').copy()
        rel_box_tcp_pos /= np.linalg.norm(rel_box_tcp_pos) + 1e-8

        return np.concatenate([box_pos, [dist_tcp_box], rel_box_tcp_pos])

    def callback_randomize_env(self):
        """ Randomize the xy coordinate of the box spawn position
        """
        box_pos = self.agent.workspace.sample()
        q_addr = self.sim.model.get_joint_qpos_addr('box:joint')
        self.qpos[q_addr[0]:q_addr[0] + 2] = [box_pos[0], box_pos[1]]

    @property
    def _reward(self):
        dist_tcp_box, _ = obj_distance(self.sim, self._tcp_id, self._box.ID)
        reward = np.double(-dist_tcp_box)

        if dist_tcp_box <= self.target_min_dist:
            reward = np.double(10.)
        return reward

    def _termination(self):
        # calculate the distance from end effector to object
        dist_tcp_box, _ = obj_distance(self.sim, self._tcp_id, self._box.ID)

        if dist_tcp_box <= self.target_min_dist:
            # terminate if end effector is close enough
            self.terminated = True
        return super()._termination()

    def step(self, action):
        gripper_gain = 1.
        return super().step(np.concatenate([action[:3], [gripper_gain]]))


class ReachEnvMocapCtrl(ReachEnvBase):
    def __init__(self, render=True, max_steps=400, nsubsteps=12, dt=2e-3, random_env=True, workspace_size='medium'):
        agent = PandaMocapControl(render)
        super().__init__(agent=agent, max_steps=max_steps, render=render, nsubsteps=nsubsteps, dt=dt,
                         random_env=random_env, workspace_size=workspace_size)


class ReachEnvJointVelCtrl(ReachEnvBase):
    def __init__(self, render=True):
        agent = PandaJointVelControl(render)
        super().__init__(agent, render)


class ReachEnvJointPosCtrl(ReachEnvBase):
    def __init__(self, render=True):
        agent = PandaJointPosControl(render)
        super().__init__(agent, render)


class ReachEnvJointTorqueCtrl(ReachEnvBase):
    def __init__(self, render=True):
        agent = PandaTorqueControl(render)
        super().__init__(agent, render)
