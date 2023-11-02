import time

import numpy as np

from alr_sim.core import logger
from alr_sim.sims.mujoco.MujocoRobot import MujocoRobot


class InteractiveMujocoRobot(MujocoRobot):
    def __init__(
        self, scene, ia_tcp_ctrl, hz=20.0, recording_file=None, *args, **kwargs
    ):
        """
        Wrapper for the MujocoRobot.
        Handles the user interaction.

        :param scene: MujocoScene
        :param ia_tcp_ctrl: Interactive TCP Controller
        :param hz: Controller Rate in hz
        :param recording_file: File to save recorded poses to. If None, no file will be written
        :param args: MujocoRobot args
        :param kwargs: MujocoRobot kwargs
        """
        super(InteractiveMujocoRobot, self).__init__(scene, *args, **kwargs)

        self.hz = hz
        self.interact_controller = ia_tcp_ctrl

        self.pose_log = PoseLog(recording_file)
        self.is_init = False

        self.init_qpos = self.get_init_qpos()
        self.init_tcp_pos = [0.55, 0.0, 0.7]
        self.init_tcp_quat = [0, 1, 0, 0]

    def _read_tcp(self):
        """
        Internal function to quickly access TCP Pos and Quat
        :return: tcp position, tcp orientation quaternion
        """
        return self.current_c_pos, self.current_c_quat

    def plot(self):
        """
        create plots. used to react to user input
        """
        self.stop_logging()
        self.robot_logger.plot(plot_selection=logger.RobotPlotFlags.JOINTS)

    def run(self):
        """
        Main Loop for controlling the Robot.

        Polls the controller for user input, especially for stopping.
        Calls internal movement_loop() function to handle movement
        """
        self.start_logging()
        self.activeController = self.gotoCartPosQuatImpedanceController
        self.gotoCartPositionAndQuat_ImpedanceCtrl(
            self.init_tcp_pos, self.init_tcp_quat, block=False
        )

        while True:
            # On Stop
            if self.interact_controller.stop():
                self.pose_log.write()
                break

            # On Move
            pos, quat, joints = self.movement_loop()

            # On Plot
            if self.interact_controller.plot():
                self.plot()

            # On Save Pose
            if self.interact_controller.save():
                self.pose_log.log(pos, quat, joints)

    def movement_loop(self):
        """
        One Iteration of the Movement Loop.
        Polls the Controller for desired position, computes the IK and drives to that configuration.
        :return: cartesian target position, cartesian target orientation, joint configuration
        """
        # Update State
        self.receiveState()
        tcp_pos, tcp_quat = self._read_tcp()
        q_pos = self.current_j_pos

        # On Reset
        if self.interact_controller.reset():
            target_pos, target_quat, target_gripper = self.reset_pose()
        # On Move
        else:
            target_pos, target_quat = self.interact_controller.move(tcp_pos, tcp_quat)
            target_gripper = self.interact_controller.grip(self.gripper_width)

        # Update Gripper state
        if np.abs(target_gripper - self.gripper_width) > 0.001:
            self.set_gripper_width = target_gripper

        self.gotoCartPosQuatImpedanceController.setDesiredPos(
            np.hstack((target_pos, target_quat))
        )
        self.gotoCartPosQuatImpedanceController.executeControllerTimeSteps(
            self, 30, block=True
        )

        self.nextStep()
        return target_pos, target_quat, self.des_joint_pos

    def reset_pose(self):
        """
        resets the robot configuration to initial positions, including helper bot
        :return: cartesian target position, cartesian target orientation, joint configuration
        """
        self.set_q(joint_pos=self.init_qpos)

        target_pos, target_quat = self.init_tcp_pos, self.init_tcp_quat
        target_gripper = 0.0

        return target_pos, target_quat, target_gripper


class PoseLog:
    def __init__(self, save_file=None):
        """
        Class for recording Poses, consisting of Cartesian Space Coordinates, Orientation
        and associated Joint Configuration

        :param save_file: Save file location. If None, no contents will be saved to disk.
        """
        self.file = save_file
        self._log = []
        self.last_ts = 0

    def log(self, pos, quat, joints):
        """
        save a pose to log
        :param pos: cartesian position
        :param quat: cartesian orientation
        :param joints: joint configuration
        :return: None
        """
        t = time.time()
        if t > self.last_ts + 1:
            self._log.append([pos, quat, joints])
            self.last_ts = t

    def write(self):
        """
        writes the pose log to the file.
        """
        if self.file is not None:
            arr = np.asarray(self._log)
            np.save(self.file, arr)
