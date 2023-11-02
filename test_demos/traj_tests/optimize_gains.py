"""
Optimize Mujoco Gains using OptunaOpt
"""

import numpy as np
import yaml

from alr_sim.sims.mujoco.MujocoCamera import MujocoCamera
from alr_sim.sims.mujoco.MujocoRobot import MujocoRobot
from alr_sim.sims.mujoco.MujocoScene import MujocoScene
from test_demos.traj_tests.follow_traj_test import mujoco_joint_gain_test
from test_demos.traj_tests.optuna_opt import OptunaOpt


class OptimizeMujocoGains:
    def __init__(
        self,
        task_name: str,
        opt_config_path: str,
        obj_func_config_path: str,
        save_to: str = None,
    ):
        """
        Args:
            task_name: task name
            opt_config_path: path to a file configuring the optimization
            obj_func_config_path: path to a file configuring the objective function
            save_to: dir to save the optimized parameters
        """

        # Use Optuna agent
        self.opt_agent = OptunaOpt(task_name, opt_config_path, self.objective, save_to)

        with open(opt_config_path + ".yaml", "r") as stream:
            opt_config = yaml.safe_load(stream)

        # parameters to be optimized.
        self.params = opt_config["params"]

        self.obj_func_config = obj_func_config_path

        self.robot = self.create_robot()

        self.params_name, self.params_range = self.params_name_range()

    @staticmethod
    def create_robot():
        """
        Create a Mujoco robot instance as well as its Scene
        Returns: robot instance

        """
        cam1 = MujocoCamera(
            cam_name="cam1", cam_pos=[0.9, 0.1, 0.9], cam_euler=[0.0, 0.9, 2.1]
        )

        object_list = []

        mujoco_scene = MujocoScene(
            object_list=object_list, camera_list=[cam1], render=False
        )

        return MujocoRobot(mujoco_scene)

    def params_name_range(self):
        """
        Decompose gains of different DOFs
        Returns: One list of gain names and one list of gain ranges (tuples)

        """
        # Keys check
        assert {"dgain", "pgain"} == set(self.params.keys()), "wrong " "parameters."

        params_name = []
        params_range = []

        for key in self.params.keys():

            assert len(list(self.params[key]["min"])) == len(
                list(self.params[key]["max"])
            ), "min and max dimension cannot match."

            for dim in range(len(list(self.params[key]["min"]))):
                params_name.append(key + str(dim + 1))
                params_range.append(
                    (self.params[key]["min"][dim], self.params[key]["max"][dim])
                )
        return params_name, params_range

    def objective(self, trial):
        """
        A wrapper objective function, which overrides the interface
        Args:
            trial: Optuna object

        Returns: a scalar error

        """
        # Get the gains to be tested
        gains = [
            trial.suggest_uniform(self.params_name[dim], *self.params_range[dim])
            for dim in range(len(self.params_name))
        ]

        # compute the scalar error
        error = mujoco_joint_gain_test(
            self.obj_func_config, self.robot, gains=gains, render=False
        )
        mean_error = np.mean(error)
        return mean_error

    def optimize(self):
        return self.opt_agent.optimize()
