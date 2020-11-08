from classic_framework.controllers import Config
from classic_framework.interface.Logger import RobotLogger


import numpy as np


class RobotBase:
    """
    This class implements a physics-engine independent robot base class.
    """

    def __init__(self, config_path, dt, num_DoF=7):
        """
        Init of the robot params.

        :param config_path: path to Config.py
        :param num_DoF: number of degrees of freedom
        """
        self.config = Config(config_path)
        self.use_inv_dyn = False
        self.gravity_comp = True
        self.clip_rate = False

        # This attribute is to fix, if we want to use the last planned spline
        # point, for re-planning another spline. If set to true, we will
        # obtain smooth splines.
        self.smooth_spline = True

        self.fictive_robot = None
        self.num_DoF = num_DoF
        self.dt = dt
        self.torque_limit = np.array([80, 80, 80, 80, 10, 10, 10],
                                     dtype=np.float64)  # Absolute torque limit
        self.controlMode = "torque"
        self.clip_actions = True
        self.rate_limit = 0.8  # Rate torque limit
        self.end_effector = None

        # torque-based controllers
        self.gotoJointController = None
        self.gotoCartPosController = None
        self.gotoCartPosQuatController = None
        self.jointTrajectoryTracker = None

        # reset all list or numpy array attributes
        self.reset_base()

    def reset_base(self):
        """
        Reset all list or numpy array attributes
        """
        # joints
        self.current_j_pos = []
        self.current_j_vel = []
        self.des_joint_pos = np.zeros((self.num_DoF,)) * np.nan
        self.des_joint_vel = np.zeros((self.num_DoF,)) * np.nan
        self.des_joint_acc = np.zeros((self.num_DoF,)) * np.nan

        # fingers
        self.current_fing_pos = []
        self.current_fing_vel = []
        self.des_fing_pos = np.zeros(2) * np.nan
        self.gripper_width = []
        self.set_gripper_width = 0.001

        # end effector
        self.current_c_pos = []
        self.current_c_vel = []
        self.current_c_quat = []
        self.current_c_quat_vel = []
        self.des_c_pos = np.zeros((3,)) * np.nan
        self.des_c_vel = np.zeros((3,)) * np.nan
        self.des_quat = np.zeros((4,)) * np.nan
        self.des_quat_vel = np.zeros((4,)) * np.nan

        # commands and forces
        self.uff = np.zeros((self.num_DoF,))
        self.uff_last = np.zeros((self.num_DoF,))
        self.last_cmd = []
        self.time_stamp = []
        self.command = np.zeros((self.num_DoF,))
        self.grav_terms = np.zeros(9) * np.nan
        self.current_load = []  # only for the joints (no fingers) for now

        self.logger = RobotLogger(self)

        self.counter = 0
        self.initCounter = 0
        self.time_stamp = 0
        self.num_calls = 0

        # torque-based controllers
        # self.gotoJointController.resetTrajectory()
        # self.gotoCartPosController.resetTrajectory()
        # self.gotoCartPosQuatController.resetTrajectory()
        # self.gotoCartPosQuatPlanningController.resetTrajectory()

    def getJacobian(self, q=None):
        raise NotImplementedError

    def getForwardKinematics(self, q=None):
        raise NotImplementedError

    def nextStep(self):
        raise NotImplementedError

    def startLogging(self):
        """
        Callback to the logger to start logging.

        :return: no return value
        """
        self.logger.isLogging = True
        self.logger.startLogging()

    def stopLogging(self):
        """
        Callback to the logger to stop logging.

        :return: no return value
        """
        self.logger.isLogging = False
        self.logger.stopLogging()

    def gotoJointPosition(self, desiredPos, duration=4.0, gains=None):
        """
        Moves the joints of the robot in the specified duration to the desired position.
        (in cartesian coordinates).

        :param desiredPos: joint values of the desired position
        :param duration: duration for moving to the position
        :param gains: gains for PD controller
        :return: no return value
        """
        self.ctrl_duration = duration

        if gains is None:  # use default gains
            data = self.config.load_yaml('PD_control_gains')
            pgain = np.array(data['pgain'], dtype=np.float64)
            dgain = np.array(data['dgain'], dtype=np.float64)
        else:  # use specified gains
            pgain = gains['pgain']
            dgain = gains['dgain']

        self.gotoJointController.trackingController.setGains(pgain, dgain)
        self.gotoJointController.setDesiredPos(desiredPos)
        self.gotoJointController.executeController(self, duration)

    def follow_JointTraj(self, desiredTraj, gains=None):
        if gains is None:  # use default gains
            data = self.config.load_yaml('PD_control_gains')
            pgain = np.array(data['pgain'], dtype=np.float64)
            dgain = np.array(data['dgain'], dtype=np.float64)
        else:  # use specified gains
            pgain = gains['pgain']
            dgain = gains['dgain']
        self.jointTrajectoryTracker.trackingController.setGains(pgain, dgain)
        self.jointTrajectoryTracker.setTrajectory(trajectory=desiredTraj)
        self.jointTrajectoryTracker.executeController(self,
                                                      maxDuration=desiredTraj.
                                                      shape[0]*self.dt)

    def gotoCartPosition(self, desiredPos, duration=4.0):
        """
        Moves the end effector of the robot in the specified duration to the desired position
        (in cartesian coordinates).

        :param desiredPos: cartesian coordinates of the desired position
        :param duration: duration for moving to the position
        :return: no return value
        """
        self.ctrl_duration = duration
        data = self.config.load_yaml('cart_ctrl_gains')
        pgain = np.array(data['cart_ctrl_pgain'], dtype=np.float64)
        dgain = np.array(data['cart_ctrl_dgain'], dtype=np.float64)
        pgain_null = np.array(data['cart_ctrl_pgain_null'], dtype=np.float64)
        dgain_null = np.array(data['cart_ctrl_dgain_null'], dtype=np.float64)
        # if self isinstance(Py)
        self.gotoCartPosController.trackingController.pgain = pgain
        self.gotoCartPosController.trackingController.dgain = dgain
        self.gotoCartPosController.trackingController.pgain_null = pgain_null
        self.gotoCartPosController.trackingController.dgain_null = dgain_null
        self.gotoCartPosController.setDesiredPos(desiredPos)
        self.gotoCartPosController.executeController(self, duration)

    def gotoCartPositionAndQuat(self, desiredPos, desiredQuat, duration=4.0):
        """
        Moves the end effector of the robot in the specified duration to the desired position
        (in cartesian coordinates) with desired orientation (given by quaternion).

        :param desiredPos: cartesian coordinates of the desired position
        :param desiredQuat: orientation given by quaternion
        :param duration: duration for moving to the position
        :return:
        """
        #TODO: Find better solution for loadin config files
        self.ctrl_duration = duration
        data = self.config.load_yaml('cartAndOr_ctrl_gains')
        pgain = np.array(data['cartAndOr_ctrl_pgain'], dtype=np.float64)
        dgain = np.array(data['cartAndOr_ctrl_dgain'], dtype=np.float64)
        pgain_null = np.array(data['cartAndOr_ctrl_pgain_null'],
                              dtype=np.float64)
        dgain_null = np.array(data['cartAndOr_ctrl_dgain_null'],
                              dtype=np.float64)
        self.gotoCartPosQuatController.trackingController.pgain = pgain
        self.gotoCartPosQuatController.trackingController.dgain = dgain
        self.gotoCartPosQuatController.trackingController.pgain_null = pgain_null
        self.gotoCartPosQuatController.trackingController.dgain_null = dgain_null
        self.gotoCartPosQuatController.setDesiredPos(
            np.hstack((desiredPos, desiredQuat)))
        self.gotoCartPosQuatController.executeController(self, duration)

    def gotoCartPositionAndQuatFictiveRobot(self, desiredPos, desiredQuat, duration=4.0):
        self.ctrl_duration = duration
        data = self.config.load_yaml('PD_control_gains')
        pgain = np.array(data['pgain'], dtype=np.float64)
        dgain = np.array(data['dgain'], dtype=np.float64)
        self.gotoJointController.trackingController.setGains(pgain, dgain)

        self.gotoCartPosQuatPlanningController.setDesiredPos(np.hstack((desiredPos, desiredQuat)))

    def get_pos_error(self, des_cart_pos):
        c_pos = self.get_end_effector_pos()
        return np.linalg.norm(np.array(c_pos) - np.array(des_cart_pos))

    def get_joint_pos_error(self, des_joint_pos):
        c_joint_pos = self.current_j_pos
        return np.linalg.norm(np.array(des_joint_pos) - np.array(c_joint_pos))

    def get_orientation_error(self, des_quat):
        return np.linalg.norm(
            np.array(self.current_c_quat) - np.array(des_quat))

    # not tested yet
    def follow_CartPositionAndQuatTraj(self, desiredTraj, desiredQuat):
        duration = desiredTraj.shape[0] * self.dt
        self.ctrl_duration = duration
        data = self.config.load_yaml('cartAndOr_ctrl_gains')
        pgain = np.array(data['cartAndOr_ctrl_pgain'], dtype=np.float64)
        dgain = np.array(data['cartAndOr_ctrl_dgain'], dtype=np.float64)
        pgain_null = np.array(data['cartAndOr_ctrl_pgain_null'],
                              dtype=np.float64)
        dgain_null = np.array(data['cartAndOr_ctrl_dgain_null'],
                              dtype=np.float64)
        self.gotoCartPosQuatController.trackingController.pgain = pgain
        self.gotoCartPosQuatController.trackingController.dgain = dgain
        self.gotoCartPosQuatController.trackingController.pgain_null = pgain_null
        self.gotoCartPosQuatController.trackingController.dgain_null = dgain_null
        self.gotoCartPosQuatController.setDesiredPos(
            np.hstack((desiredTraj, desiredQuat)))
        self.gotoCartPosQuatController.executeController(self, duration)

    def fing_ctrl_step(self):
        """
        Calculates the control for the robot finger joints.

        :return: controlling for the robot finger joints with dimension: (num_finger_joints, )
        """
        # pgain = 1 * np.array([200, 200], dtype=np.float64)
        pgain = 1 * np.array([500, 500], dtype=np.float64)
        # dgain = np.sqrt(pgain) #1 * np.array([10, 10], dtype=np.float64)
        dgain = 1 * np.array([10, 10],
                             dtype=np.float64)  # 1 * np.array([10, 10], dtype=np.float64)

        gripper_width = self.current_fing_pos
        gripper_width_vel = self.current_fing_vel

        des_fing_pos = np.array(
            [self.set_gripper_width, self.set_gripper_width], dtype=np.float64)
        return pgain * (
                des_fing_pos - gripper_width) - dgain * gripper_width_vel

    def get_end_effector_pos(self):
        return self.current_c_pos

    def receiveState(self):
        raise NotImplementedError

    def nextStep(self):
        raise NotImplementedError

    def get_command_from_inverse_dynamics(self, target_j_acc, mj_calc_inv=False,
                                          robot_id=None, client_id=None):
        # Do not forget to do a distuinction for mujoco..... if we only do want to have gravity comp,
        # then we will not call the inverse dynamics function.... only sim.data.qfrc_bias !!!
        raise NotImplementedError

    def preprocessCommand(self, target_j_acc):
        if len(target_j_acc.shape) == 2:
            target_j_acc = target_j_acc.reshape((-1))
        if target_j_acc.shape[0] != self.num_DoF:
            raise ValueError(
                'Specified motor command vector needs to be of size %d\n',
                target_j_acc.shape[0])

        finger_commands = self.fing_ctrl_step()
        if self.use_inv_dyn:
            target_j_acc = np.append(target_j_acc, finger_commands[0])
            target_j_acc = np.append(target_j_acc, finger_commands[1])
            self.des_joint_acc = target_j_acc.copy()
            self.uff = self.get_command_from_inverse_dynamics(target_j_acc,
                                                              mj_calc_inv=True)
            self.uff[7:9] = finger_commands

        else:
            target_j_acc = np.append(target_j_acc, finger_commands[0])
            target_j_acc = np.append(target_j_acc, finger_commands[1])
            if self.gravity_comp:
                comp_forces = self.get_command_from_inverse_dynamics(
                    target_j_acc=np.zeros(9), mj_calc_inv=False)
                self.grav_terms = comp_forces.copy()
                self.uff = comp_forces + target_j_acc
            else:
                self.uff = target_j_acc

        # Catch start of control
        if self.initCounter == 0:
            self.uff_last = 0

        if self.clip_rate:
            # Clip and rate limit torque
            uff_diff = self.uff - self.uff_last
            uff_diff = np.clip(uff_diff, - self.rate_limit, self.rate_limit)
            self.uff = self.uff_last + uff_diff

        if self.clip_actions:
            self.uff[:7] = np.clip(self.uff[:7], - self.torque_limit,
                                   self.torque_limit)

    def beam_to_cart_pos_and_quat(self, desiredPos, desiredQuat):
        raise NotImplementedError

    def beam_to_joint_pos(self, desiredJoints):
        raise NotImplementedError

    # def get_c_pos(self):
    #     if isinstance(self, MuJoCoRobot) or isinstance(self, MujocoFictiveRobot):
    #         if self.end_effector == 'tcp':
    #             return self.current_c_pos_tcp
    #         else:
    #             return self.current_c_pos
    #     else:
    #         return self.current_c_pos



