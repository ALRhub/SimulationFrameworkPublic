from abc import ABC, abstractmethod

import gym
import numpy as np
from gym.utils import seeding
from mujoco_py import MjSimState, MjViewer

from classic_framework.mujoco.mujoco_utils import MujocoSceneParser, MujocoLoadable
from classic_framework.mujoco.mujoco_utils.mujoco_scene_object import MujocoWorkspace
from gym_framework.panda_ctrl.panda_mujoco_base_ctrl import PandaBase


class MujocoEnv(gym.Env, ABC):
    """Open AI gym_envs environment for tasks using a Panda Franka robot arm with MuJoCo physics.
    """
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self,
                 agent: PandaBase,
                 xml_path='./envs/mujoco/panda/panda_gym.xml',
                 obj_list=[],
                 max_steps=2000,
                 nsubsteps=1,
                 dt=2e-3,
                 robot_init_qpos=None,
                 random_env=True,
                 render=True,
                 workspace_size='medium'):
        """
        Args:
            max_steps:
                Maximum number of steps per episode
        """
        self.terminated = False
        self.env_step_counter = 0
        self.max_steps = max_steps
        self.episode = 0

        self.xml_parser = MujocoSceneParser(panda_xml_path=xml_path)
        self.xml_parser.set_dt(dt)

        obj_list.append(MujocoWorkspace(workspace_size))

        self.xml_parser.set_control(control=agent, set_gripper=False)

        # adding objects to the scene
        for obj in obj_list:
            if isinstance(obj, MujocoLoadable):
                self.xml_parser.load_mj_loadable(obj)

        self._tcp_id = 'tcp'
        self.sim, _ = self.xml_parser.create_sim(nsubsteps)
        self.viewer = MjViewer(self.sim)
        self.agent = agent
        self.agent.env_setup(self.sim, self.viewer)
        self.action_space = self.agent.action_space

        self.qpos = None

        # Specify initial joint position for panda robot
        if robot_init_qpos is None:
            robot_init_qpos = np.array([4.96976216e-05, - 1.84996010e-01, - 4.63546468e-05, - 2.08528506e+00,
                                        -8.95942123e-06, 1.90028276e+00, 7.85404067e-01, 0.04, 0.04])
        self.robot_init_qpos = robot_init_qpos

        self.random_env = random_env

        self.reset()

    def step(self, action):
        """Run one timestep of the environment's dynamics. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state.

        Accepts an action and returns a tuple (observation, reward, done, info).

        Args:
            action (object): an action provided by the agent

        Returns:
            observation (object): agent's observation of the current environment
            reward (float) : amount of reward returned after previous action
            done (bool): whether the episode has ended, in which case further step() calls will return undefined results
            info (dict): contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)
        """
        self.agent.apply_action(action)
        observation = self.get_observations()
        reward = self._reward
        done = self._termination()
        if not done:
            self.env_step_counter += 1
        return np.array(observation), reward, done, {}

    def reset(self):
        """Resets the environment (including the agent) to the initial conditions.
        """
        self.sim.reset()
        # Set initial position and velocity
        self.qpos = self.sim.data.qpos.copy()
        self.qpos[:self.robot_init_qpos.shape[0]] = self.robot_init_qpos
        if self.random_env:
            self.callback_randomize_env()
        qvel = np.zeros(self.sim.data.qvel.shape)
        mjSimState = MjSimState(time=0.0, qpos=self.qpos, qvel=qvel, act=None, udd_state={})
        self.sim.set_state(mjSimState)
        self.sim.forward()

        self.terminated = False
        self.env_step_counter = 0

        return self.get_observations()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _termination(self):
        """Checks if the robot either exceeded the maximum number of steps or is terminated according to another task
        dependent metric.

        Returns:
            True if robot should terminate
        """
        if self.terminated or self.env_step_counter >= self.max_steps - 1:
            self.terminated = False
            self.env_step_counter = 0
            self.episode += 1
            return True
        return False

    def get_observations(self):
        return np.concatenate((self.agent.state, self.environment_observations))

    @property
    def observation_space(self):
        agent_obs_low, agent_obs_high = self.agent.state_bounds
        env_obs_low, env_obs_high = self.observation_bounds
        obs_space_low = np.concatenate([agent_obs_low, env_obs_low])
        obs_space_high = np.concatenate([agent_obs_high, env_obs_high])
        return gym.spaces.Box(low=obs_space_low, high=obs_space_high)

    @property
    def observation_bounds(self):
        """
        Defines the observations specific to the environment (e.g. object positions).
        """
        low = -np.inf * np.ones_like(self.environment_observations)
        high = np.inf * np.ones_like(self.environment_observations)
        return low, high

    def observation_normalization(self, observation, lower=-1, upper=1):
        """ Normalized the observations which are in [observation_space.low, observation_space.high] to [lower, upper].
        """
        return (self.observation_space.high - self.observation_space.low) * (
                (observation - lower) / (upper - lower)) + self.observation_space.low

    @property
    def observation_dim(self):
        return self.get_observations().shape[0]

    @property
    def action_dim(self):
        return self.agent.action_dimension

    @property
    @abstractmethod
    def environment_observations(self):
        """
        Defines the observations specific to the environment (e.g. object positions).
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def _reward(self):
        """Calculates the reward which is based on the distance of the end effector from the object.

        Returns:
            Scalar reward.
        """
        raise NotImplementedError

    @abstractmethod
    def callback_randomize_env(self):
        """
        Randomizes the initial positions of objects in the simulation. Implementation is environment specific.
        """
        raise NotImplementedError
