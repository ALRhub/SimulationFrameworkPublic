"""
An Optuna optimization agent
"""
import datetime
import json
from pathlib import Path

import optuna
import yaml


class OptunaOpt:
    def __init__(
        self,
        task_name: str,
        opt_config: str,
        objective,
        save_to: str = None,
    ):
        """
        Interface class using Optuna
        Args:
            task_name: A meaningful name used to save the optimized parameters.
            opt_config: A file with optimization configuration
            objective: an objective function to be optimized
            save_to: a dir where the optimized parameters should be saved
        """

        assert callable(objective), "objective is not a callable function."
        self.objective = objective

        self.save_to = save_to

        self.task_name = task_name
        self.optimize_config = opt_config

        with open(self.optimize_config + ".yaml", "r") as stream:
            opt_config = yaml.safe_load(stream)

        self.optimizer = opt_config["optimizer"]

        # num of calls of the optimizer
        self.n_trials = opt_config["n_trials"]

        # num of jobs
        # TODO: debug in case of more than 1 job.
        self.n_jobs = opt_config["n_jobs"]

    def optimize(self):
        """
        Optimize the objective function, given all the configuration
        Returns: best params
        """

        if self.optimizer == "cma_es":
            sampler = optuna.samplers.CmaEsSampler()
        elif self.optimizer == None:
            sampler = None  # default sampler
        else:
            raise NotImplementedError

        self.study = optuna.create_study(sampler=sampler)
        self.study.optimize(
            self.objective,
            n_trials=self.n_trials,
            n_jobs=self.n_jobs,
            show_progress_bar=False,
        )

        best_param = self.study.best_params

        print("best_param: ", best_param, sep="\n")

        if self.save_to is not None:
            self.save_to_file()

        return self.study.best_params

    def save_to_file(self):
        """
        Save best_param to json
        Returns: None

        """
        f_time = "-%Y-%m-%d_%H-%M-%S"
        try:
            # save to given path
            formatted_name = (
                self.save_to
                + "/"
                + self.task_name
                + datetime.datetime.now().strftime(f_time)
                + ".json"
            )
            formatted_name = str(Path(formatted_name).resolve())
            with open(formatted_name, "w") as fp:
                json.dump(self.study.best_params, fp)

        except IOError:
            # given path doesn't exist, thus save to default path
            module_path = str(Path(__file__).resolve().parents[0])

            formatted_name = (
                module_path
                + "/test_results/"
                + self.task_name
                + datetime.datetime.now().strftime(f_time)
                + ".json"
            )
            with open(formatted_name, "w") as fp:
                json.dump(self.study.best_params, fp)

        print("Parameters have been saved to " + formatted_name)
