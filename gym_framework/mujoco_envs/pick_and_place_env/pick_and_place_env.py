import numpy as np

from gym_framework.mujoco_envs.mujoco_env import MujocoEnv
from gym_framework.mujoco_objects import env_objects
from gym_framework.panda_ctrl.panda_mujoco_mocap_ctrl import PandaMocapControl
from gym_framework.utils.helper import has_collision, obj_distance


class PickAndPlaceBase(MujocoEnv):
    """
    Reach task: The agent is asked to go to a certain object and gets reward when getting closer to the object. Once
    the object is reached, the agent gets a high reward.
    """
    def __init__(self, agent, render=True, max_steps=2000,
                 nsubsteps=1, dt=2e-3, random_env=True, workspace_size='medium'):
        self._box = env_objects.Box()
        self._goal = env_objects.Goal()
        super().__init__(agent=agent, obj_list=[self._goal, self._box], max_steps=max_steps, render=render,
                         nsubsteps=nsubsteps, dt=dt, random_env=random_env, workspace_size=workspace_size)
        self.target_min_dist = 0.03

    @property
    def environment_observations(self):
        """ Defines all the states specific for the pick and place environment.
        """
        tcp_pos = self.sim.data.get_body_xpos('tcp').copy()
        box_pos = self.sim.data.get_body_xpos('box').copy()
        goal_pos = self.sim.data.get_site_xpos('goal:site1').copy()
        box_quat = self.sim.data.get_body_xquat('box').copy()
        dist_box_tcp = [np.linalg.norm(tcp_pos - box_pos)]
        dist_goal_tcp = [np.linalg.norm(tcp_pos - goal_pos)]
        dist_box_goal = [np.linalg.norm(goal_pos - box_pos)]
        rel_box_tcp_pos = box_pos - self.sim.data.get_body_xpos('tcp').copy()
        rel_box_tcp_pos /= np.linalg.norm(rel_box_tcp_pos) + 1e-8
        rel_goal_tcp_pos = goal_pos - self.sim.data.get_body_xpos('tcp').copy()
        rel_goal_tcp_pos /= np.linalg.norm(rel_goal_tcp_pos) + 1e-8
        rel_box_goal_pos = goal_pos - self.sim.data.get_body_xpos('tcp').copy()
        rel_box_goal_pos /= np.linalg.norm(rel_box_goal_pos) + 1e-8
        touch_right = [int(has_collision('touch_right', 'box:geom1', self.sim))]
        touch_left = [int(has_collision('touch_left', 'box:geom1', self.sim))]
        return np.concatenate([box_pos, goal_pos, box_quat, dist_box_tcp, dist_goal_tcp, dist_box_goal,
                               rel_box_tcp_pos, rel_goal_tcp_pos, rel_box_goal_pos, touch_left, touch_right])

    def callback_randomize_env(self):
        """ Randomize the xy coordinate of the box spawn position
        """
        # Randomize the box position
        box_pos = self.agent.workspace.sample()
        q_addr = self.sim.model.get_joint_qpos_addr('box:joint')
        self.qpos[q_addr[0]:q_addr[0] + 2] = [box_pos[0], box_pos[1]]

        # Randomize the goal position
        site_offset = self.sim.model.site_pos[self.sim.model.site_name2id('goal:site1')]
        goal_pos = (self.agent.workspace.sample() - site_offset)
        goal_pos[2] = max(goal_pos[2], 0.1)  # Ensure that the goal is in the air
        body_id = self.sim.model.body_name2id('goal')
        self.sim.model.body_pos[body_id] = goal_pos

    @property
    def _reward(self):
        dist_tcp_box, _ = obj_distance(self.sim, self._tcp_id, self._box.ID)
        dist_box_goal, _ = obj_distance(self.sim, self._box.ID, self._goal.ID)
        touch_right = has_collision('touch_right', 'box:geom1', self.sim)
        touch_left = has_collision('touch_left', 'box:geom1', self.sim)
        return -10 * (dist_tcp_box + dist_box_goal) + 0.25 * (touch_left + touch_right)

    def _termination(self):
        """Checks if the box is close enough to the goal. If the condition is satisfied, the episode is terminated.
        """
        dist, _ = obj_distance(self.sim, self._box.ID, self._goal.ID)
        if dist <= self.target_min_dist:
            print("success")
        self.terminated = True if dist <= self.target_min_dist else False
        return super()._termination()


class PickAndPlaceMocapCtrl(PickAndPlaceBase):
    def __init__(self, render=True, max_steps=2000, nsubsteps=1, dt=2e-3, random_env=True, workspace_size='medium'):
        agent = PandaMocapControl(render)
        super().__init__(agent=agent, max_steps=max_steps, render=render, nsubsteps=nsubsteps, dt=dt,
                         random_env=random_env, workspace_size=workspace_size)
