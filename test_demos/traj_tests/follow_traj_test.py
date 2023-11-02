# A test factory for different controller types and robots in trajectory
# following tests.

# TODO:
#  1. Add more trajectories.
#  2. Add more different controllers types.

import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml

from alr_sim.core.logger import RobotPlotFlags
from alr_sim.core.Robots import RobotBase
from alr_sim.sims.mujoco.MujocoCamera import MujocoCamera
from alr_sim.sims.mujoco.MujocoRobot import MujocoRobot
from alr_sim.sims.mujoco.MujocoScene import MujocoScene


# matplotlib.use('TkAgg')


class FollowTrajTest:
    """
    Class follow trajectory test
    """

    def __init__(self, robot: RobotBase, config, gains=None):
        """
        Constructor
        Args:
            robot: a robot instance
            config: config file of the test and trajectory
            gains: gains of the controller
        """
        self.module_path = str(Path(__file__).resolve().parents[0])
        self.joint_config_path = "/test_config/joint_range.yaml"

        self.space = config["space"]
        assert self.space in {"joint", "CartPosQuat"}, "wrong tracking type."

        self.traj_type = config["traj_type"]
        duration = config["duration"]
        dt = config["dt"]
        dofs = config["dofs"]
        traj_shape = config["traj_shape"]
        period = config["period"]
        phase = config["phase"]
        magnitude = config["magnitude"]
        self.robot = robot
        joint_ranges = self.compute_joint_range(magnitude)
        self.gains = gains
        self.tracking_error = None
        self.desired_traj = self.get_traj(
            self.traj_type,
            duration=duration,
            dt=dt,
            dofs=dofs,
            traj_shape=traj_shape,
            period=period,
            phase=phase,
            magnitude=magnitude,
            joint_ranges=joint_ranges,
        )

    def get_traj(self, traj_type="dummy", **args):
        """
        Get the trajectory from file or from code
        Args:
            traj_type: type of the trajectory, "dummy" or "file"
            **args: dict of arguments specifying the trajectory

        Returns: trajectory

        """
        if traj_type == "dummy":
            return self.get_dummy_traj(**args)
        elif traj_type == "file":
            return self.get_file_traj(**args)
        else:
            raise ValueError("Error, wrong type of trajectory.")

    def get_dummy_traj(self, **args):
        """
        Dummy trajectory factory
        Args:
            **args: dict of arguments specifying the trajectory

        Returns: a dummy trajectory in the form of numpy array

        """
        assert {"traj_shape", "duration", "dt", "dofs"}.issubset(
            set(args.keys())
        ), "Lack of trajectory arguments."

        time_seq = np.arange(0.0, args["duration"], args["dt"])

        if args["traj_shape"] == "sin":
            return self.get_sin_trajectory(time_seq, **args)
        else:
            raise NotImplementedError

    @staticmethod
    def get_sin_trajectory(time_seq, **args):
        """
        Generate a sin trajectory
        Args:
            time_seq: different time points
            **args: dict of arguments for sin trajectory

        Returns: a sin trajectory in the form of numpy array [time, dofs]

        """
        assert {"joint_ranges", "period", "phase"}.issubset(
            set(args.keys())
        ), "Lack of arguments to define a sine trajectory."
        traj = np.zeros(shape=(time_seq.shape[0], args["dofs"]))
        for dof in range(args["dofs"]):
            j_range = args["joint_ranges"][dof][1] - args["joint_ranges"][dof][0]
            j_shift = (args["joint_ranges"][dof][1] + args["joint_ranges"][dof][0]) / 2
            traj[:, dof] = j_shift + j_range / 2 * np.sin(
                (time_seq + args["phase"]) * 2 * math.pi / args["period"]
            )
        return traj

    def compute_joint_range(self, magnitude):
        """
        Get the valid range of the joint to help generate the dummy trajectory
        Args:
            magnitude: a ratio coefficient

        Returns: joint ranges

        """
        with open(self.module_path + self.joint_config_path, "r") as stream:
            joint_config = yaml.safe_load(stream)
        joint_limits = np.zeros(shape=(7, 2))
        for dof in range(7):
            joint_name = "joint{}".format(dof + 1)
            joint_limits[dof] = np.asarray(joint_config[joint_name])
        joint_avgs = np.mean(joint_limits, axis=1, keepdims=True)
        joint_ranges = magnitude * (joint_limits + joint_avgs)

        return joint_ranges

    def get_file_traj(self, **args):
        """
        File trajectory factory
        Args:
            **args: arguments dict specifying the file trajectory

        Returns: a file trajectory as numpy array [time, dofs]

        """
        raise NotImplementedError

    @staticmethod
    def load_traj():
        """
        Load trajectory from a file
        Returns: a file trajectory as numpy array [time, dofs]

        """
        raise NotImplementedError

    def get_error(self, plotting=False, start_time=None):
        """
        Get the error at different time and DOFs
        Args:
            plotting: true if plotting the error figures
            start_time: return the error starting from this time.

        Returns: error as numpy array [time, dof]

        """
        start_index = 0
        if start_time is not None:
            assert 0 <= start_time, "wrong start time."
            start_index = start_time * 1.0 / self.robot.dt

        error = (
            self.robot.robot_logger.des_joint_pos - self.robot.robot_logger.joint_pos
        ) ** 2

        if plotting:
            joint_pos_fig = plt.figure()
            for k in range(7):
                plt.figure(joint_pos_fig.number)
                plt.subplot(7, 1, k + 1)
                plt.plot(error[:, k])
            plt.title(" joint get_error ")
            plt.show()

            self.robot.robot_logger.plot(RobotPlotFlags.JOINTS)
        assert start_index < error.shape[0]
        return error[int(start_index) :]

    def test(self, plotting=False):
        """
        Test function, do the main job
        Args:
            plotting: true if plotting the figures

        Returns: tracking_error as numpy array

        """
        self.robot.scene.start_logging()
        goto_duration = 4.0
        if self.space == "joint":
            self.robot.gotoJointPosition(
                self.desired_traj[0], duration=goto_duration, gains=self.gains
            )
            self.robot.follow_JointTraj(self.desired_traj, gains=self.gains)
            self.robot.scene.stop_logging()
            self.tracking_error = self.get_error(
                start_time=goto_duration + 1.0, plotting=plotting
            )
            return self.tracking_error

        else:  # CartPositionAndQuat
            raise NotImplementedError

    def reset(self):
        """
        Reset the robot
        Returns: None

        """
        self.robot.scene.sim.reset()

    # End of class definition


def mujoco_joint_gain_test(
    test_config_file: str,
    mujoco_robot: MujocoRobot = None,
    gains: list = None,
    render: bool = False,
    plotting: bool = False,
) -> np.ndarray:
    """
    A test function defining a joint test instance in Mujoco
    Args:
        test_config_file: file configuring the joint test
        mujoco_robot: an robot instance
        gains: The gain to be tested for. None if using the default gains
            otherwise
        render: visualize the simulation or not
        plotting: plotting the error figures or not

    Returns: the test error as a numpy array [time, dof]

    """
    with open(test_config_file + ".yaml", "r") as stream:
        test_config = yaml.safe_load(stream)

    if mujoco_robot is None:
        cam1 = MujocoCamera(
            cam_name="cam1", cam_pos=[0.9, 0.1, 0.9], cam_euler=[0.0, 0.9, 2.1]
        )
        object_list = []
        mujoco_scene = MujocoScene(
            object_list=object_list, camera_list=[cam1], render=render
        )
        mujoco_robot = MujocoRobot(mujoco_scene, clip_actions=True)

    else:
        mujoco_robot.reset()

    if gains is not None:
        gains = {"pgain": np.asarray(gains[-7:]), "dgain": np.asarray(gains[:7])}

    ft_test = FollowTrajTest(mujoco_robot, config=test_config, gains=gains)
    test_error = ft_test.test(plotting)

    mujoco_robot.scene.sim.reset()

    return test_error  # may contain nan in velocities between two
    # trajectories
