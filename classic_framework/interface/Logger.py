import numpy as np
from enum import Flag, auto

class LoggerBase:
    def __init__(self, object):
        """
        Initialization of the quantities to log.
        """
        pass

    def startLogging(self):
        """
        Starts logging.

        :return: no return value
        """
        raise NotImplementedError

    def logData(self):
        """
        Appends current state of each logged value.

        :param robot: instance of the robot
        :return: no return value
        """
        raise NotImplementedError

    def stopLogging(self):
        raise NotImplementedError


class RobotPlotFlags(Flag):
    JOINT_POS = auto()
    JOINT_VEL = auto()
    JOINT_ACC  = auto()
    CART_POS = auto()
    CART_VEL = auto()
    ORIENTATION = auto()
    ORIENTATION_VEL = auto()
    COMMAND = auto()
    TORQUES = auto()
    GRAVITY = auto()
    LOADS = auto()
    TIME_STAMPS = auto()

    GRIPPER_POS = auto()
    GRIPPER_WIDTH = auto()
    GRIPPER_FORCE = auto()

    GRIPPER = GRIPPER_WIDTH | GRIPPER_POS | GRIPPER_FORCE
    JOINTS = JOINT_POS | JOINT_ACC | JOINT_VEL
    END_EFFECTOR = CART_POS | CART_VEL | ORIENTATION | ORIENTATION_VEL


class RobotLogger(LoggerBase):
    """
    Logger for the
    - joint positions
    - joint velocities
    - cartesian positions
    - cartesian velocities
    - orientations in w, x, y, z
    - controller commands (without gravity comp)
    - (clipped) commands with gravity
    - gravity compensation terms
    - joint accelerations
    - joint loads

    Resets all logs by calling :func:`startLogging`. Appends current state of each quantity by calling :func:`logData`.
    Stops logging by calling :func:`stopLogging` and plots data by calling :func:`plot`.

    :return: no return value
    """

    def __init__(self, robot):
        super().__init__(robot)
        """
        Initialization of the quantities to log.
        """

        self.isLogging = False

        # joints
        self.joint_pos_list = []
        self.joint_vel_list = []
        self.des_joint_pos_list = []
        self.des_joint_vel_list = []
        self.des_joint_acc_list = []

        # fingers
        self.finger_pos_list = []
        self.finger_vel_list = []
        self.des_finger_pos_list = []
        self.gripper_width_list = []

        # end effector
        self.cart_pos_list = []
        self.cart_vel_list = []
        self.cart_quat_list = []
        self.cart_quat_vel_list = []
        self.des_c_pos_list = []
        self.des_c_vel_list = []
        self.des_quat_list = []
        self.des_quat_vel_list = []

        # commands and forces
        self.uff_list = []
        self.last_cmd_list = []
        self.time_stamp_list = []
        self.command_list = []
        self.grav_term_list = []
        self.load_list = []

        self.maxTimeSteps = 5000000000000000
        self.robot = robot

    def startLogging(self):
        """
        Starts logging.

        :return: no return value
        """
        # joints
        self.joint_pos_list = []
        self.joint_vel_list = []
        self.des_joint_pos_list = []
        self.des_joint_vel_list = []
        self.des_joint_acc_list = []

        # fingers
        self.finger_pos_list = []
        self.finger_vel_list = []
        self.des_finger_pos_list = []
        self.gripper_width_list = []

        # end effector
        self.cart_pos_list = []
        self.cart_vel_list = []
        self.cart_quat_list = []
        self.cart_quat_vel_list = []
        self.des_c_pos_list = []
        self.des_c_vel_list = []
        self.des_quat_list = []
        self.des_quat_vel_list = []

        # commands and forces
        self.uff_list = []
        self.last_cmd_list = []
        self.time_stamp_list = []
        self.command_list = []
        self.grav_term_list = []
        self.load_list = []

    def logData(self):
        """
        Appends current state of each logged value.

        :param robot: instance of the robot
        :return: no return value
        """
        robot = self.robot

        # joints
        self.joint_pos_list.append(robot.current_j_pos)
        self.joint_vel_list.append(robot.current_j_vel)
        self.des_joint_pos_list.append(robot.des_joint_pos)
        self.des_joint_vel_list.append(robot.des_joint_vel)
        self.des_joint_acc_list.append(robot.des_joint_acc)

        # fingers
        self.finger_pos_list.append(robot.current_fing_pos)
        self.finger_vel_list.append(robot.current_fing_vel)
        self.des_finger_pos_list.append(robot.set_gripper_width)
        self.gripper_width_list.append(robot.gripper_width)

        # end effector
        self.cart_pos_list.append(robot.get_end_effector_pos())
        self.cart_vel_list.append(robot.current_c_vel)
        self.cart_quat_list.append(robot.current_c_quat)
        self.cart_quat_vel_list.append(robot.current_c_quat_vel)
        self.des_c_pos_list.append(robot.des_c_pos)
        self.des_c_vel_list.append(robot.des_c_vel)
        self.des_quat_list.append(robot.des_quat)
        self.des_quat_vel_list.append(robot.des_quat_vel)

        # commands and forces
        self.uff_list.append(robot.uff.copy())
        self.last_cmd_list.append(robot.last_cmd)
        self.time_stamp_list.append(robot.time_stamp)
        self.command_list.append(robot.command)
        self.grav_term_list.append(robot.grav_terms)
        self.load_list.append(robot.current_load)

        if len(self.joint_pos_list) > self.maxTimeSteps:
            #joints
            self.joint_pos_list.pop(0)
            self.joint_vel_list.pop(0)
            self.des_joint_pos_list.pop(0)
            self.des_joint_vel_list.pop(0)
            self.des_joint_acc_list.pop(0)

            # fingers
            self.des_finger_pos_list.pop(0)
            self.finger_vel_list.pop(0)
            #self.des_finger_pos_list.pop(0)
            self.gripper_width_list.pop(0)

            # end effector
            self.cart_pos_list.pop(0)
            self.cart_vel_list.pop(0)
            self.cart_quat_list.pop(0)
            self.des_c_pos_list.pop(0)
            self.des_c_vel_list.pop(0)
            self.des_quat_list.pop(0)
            self.des_quat_vel_list.pop(0)

            # commands and forces
            self.uff_list.pop(0)
            self.last_cmd_list.pop(0)
            self.time_stamp_list.pop(0)
            self.command_list.pop(0)
            self.grav_term_list.pop(0)
            self.load_list.pop(0)

    def stopLogging(self):
        """
        Stops logging.

        :return: No return value
        """

        self.isLogging = False
        if len(self.time_stamp_list) > 0:

            # joints
            self.joint_pos = np.array(self.joint_pos_list)
            self.joint_vel = np.array(self.joint_vel_list)
            self.des_joint_pos = np.array(self.des_joint_pos_list)
            self.des_joint_vel = np.array(self.des_joint_vel_list)
            self.des_joint_acc = np.array(self.des_joint_acc_list)

            # fingers
            self.finger_pos = np.array(self.finger_pos_list)
            self.finger_vel = np.array(self.finger_vel_list)
            self.des_finger_pos = np.array(self.des_finger_pos_list)
            self.gripper_width = np.array(self.gripper_width_list)

            # end effector
            self.cart_pos = np.array(self.cart_pos_list)
            self.cart_vel = np.array(self.cart_vel_list)
            self.cart_quat = np.array(self.cart_quat_list)
            self.cart_quat_vel = np.array(self.cart_quat_vel_list)
            self.des_c_pos = np.array(self.des_c_pos_list)
            self.des_c_vel = np.array(self.des_c_vel_list)
            self.des_quat = np.array(self.des_quat_list)
            self.des_quat_vel = np.array(self.des_quat_vel_list)

            # commands and forces
            self.uff = np.array(self.uff_list)
            self.last_cmd = np.array(self.last_cmd_list)
            self.command = np.array(self.command_list)
            self.grav_terms = np.array(self.grav_term_list)
            self.time_stamp = np.array(self.time_stamp_list)
            self.time_stamp = self.time_stamp - self.time_stamp[0]
            self.load = np.array(self.load_list)

    def plot(self, plotSelection = RobotPlotFlags.JOINTS):
        """
        Plots the
        - joint positions
        - joint velocities
        - cartesian positions
        - cartesian velocities
        - orientations in wxyz
        - controller commands (without gravity comp)
        - (clipped) commands with gravity
        - gravity compensation terms
        - joint accelerations
        - joint loads

        :return: no return value
        """
        import matplotlib
        import matplotlib.pyplot as plt
        #matplotlib.use('TkAgg')
        if self.isLogging:
            self.stopLogging()

        robot_acceleration = np.diff(self.joint_vel, 1, axis=0) / self.robot.dt
        j1_limit_lower = -2.9671
        j1_limit_upper = 2.9671
        j2_limit_lower = -1.8326
        j2_limit_upper = 1.8326
        j3_limit_lower = -2.9671
        j3_limit_upper = 2.9671
        j4_limit_lower = -3.1416
        j4_limit_upper = 0.0873
        j5_limit_lower = -2.9671
        j5_limit_upper = 2.9671
        j6_limit_lower = -0.0873
        j6_limit_upper = 3.8223
        j7_limit_lower = -2.9671
        j7_limit_upper = 2.9671

        j1_limit_lower_safety = -2.8973
        j1_limit_upper_safety = 2.8973
        j2_limit_lower_safety = -1.7628
        j2_limit_upper_safety = 1.7628
        j3_limit_lower_safety = -2.8973
        j3_limit_upper_safety = 2.8973
        j4_limit_lower_safety = -3.0718
        j4_limit_upper_safety = 0.0175
        j5_limit_lower_safety = -2.8973
        j5_limit_upper_safety = 2.8973
        j6_limit_lower_safety = -0.0175
        j6_limit_upper_safety = 3.7525
        j7_limit_lower_safety = -2.8973
        j7_limit_upper_safety = 2.8973

        j1_vel_limit_lower = -2.3925
        j1_vel_limit_upper = 2.3925
        j2_vel_limit_lower = -2.3925
        j2_vel_limit_upper = 2.3925
        j3_vel_limit_lower = -2.3925
        j3_vel_limit_upper = 2.3925
        j4_vel_limit_lower = -2.3925
        j4_vel_limit_upper = 2.3925
        j5_vel_limit_lower = -2.8710
        j5_vel_limit_upper = 2.8710
        j6_vel_limit_lower = -2.8710
        j6_vel_limit_upper = 2.8710
        j7_vel_limit_lower = -2.8710
        j7_vel_limit_upper = 2.8710

        # joint limits in list
        upper_limits = [j1_limit_upper, j2_limit_upper, j3_limit_upper,
                        j4_limit_upper, j5_limit_upper, j6_limit_upper,
                        j7_limit_upper]
        lower_limits = [j1_limit_lower, j2_limit_lower, j3_limit_lower,
                        j4_limit_lower, j5_limit_lower, j6_limit_lower,
                        j7_limit_lower]

        soft_upper_limits = [j1_limit_upper_safety, j2_limit_upper_safety, j3_limit_upper_safety,
                             j4_limit_upper_safety, j5_limit_upper_safety, j6_limit_upper_safety,
                             j7_limit_upper_safety]

        soft_lower_limits = [j1_limit_lower_safety, j2_limit_lower_safety, j3_limit_lower_safety,
                             j4_limit_lower_safety, j5_limit_lower_safety, j6_limit_lower_safety,
                             j7_limit_lower_safety]

        # joint velocity limits in list
        upper_limits_vel = [j1_vel_limit_upper, j2_vel_limit_upper, j3_vel_limit_upper,
                            j4_vel_limit_upper, j5_vel_limit_upper, j6_vel_limit_upper,
                            j7_vel_limit_upper]

        lower_limits_vel = [j1_vel_limit_lower, j2_vel_limit_lower, j3_vel_limit_lower,
                            j4_vel_limit_lower, j5_vel_limit_lower, j6_vel_limit_lower,
                            j7_vel_limit_lower]

        if RobotPlotFlags.JOINT_POS in plotSelection:
            joint_pos_fig = plt.figure()
            for k in range(7):
                plt.figure(joint_pos_fig.number)
                plt.subplot(7, 1, k + 1)
                plt.plot(self.time_stamp, self.joint_pos[:, k])
                plt.plot(self.time_stamp, self.des_joint_pos[:, k], 'r')
                # joint limits:
                #plt.plot(np.ones(len(self.joint_pos[:, k])) * upper_limits[k], 'r--', linewidth=0.7)
                #plt.plot(np.ones(len(self.joint_pos[:, k])) * soft_upper_limits[k], 'g--', linewidth=0.7)
                #plt.plot(np.ones(len(self.joint_pos[:, k])) * lower_limits[k], 'r--', linewidth=0.7)
                #plt.plot(np.ones(len(self.joint_pos[:, k])) * soft_lower_limits[k], 'g--', linewidth=0.7)
            plt.title(' joint positions ')

        if RobotPlotFlags.JOINT_VEL in plotSelection:
            joint_vel_fig = plt.figure()
            for k in range(7):
                plt.figure(joint_vel_fig.number)
                plt.subplot(7, 1, k + 1)
                plt.plot(self.time_stamp, self.joint_vel[:, k])
                plt.plot(self.time_stamp, self.des_joint_vel[:, k], 'r')
                #plt.plot(np.ones(len(self.des_joint_vel[:, k])) * upper_limits_vel[k], 'r--', linewidth=0.7)
                #plt.plot(np.ones(len(self.des_joint_vel[:, k])) * lower_limits_vel[k], 'r--', linewidth=0.7)
            plt.title(' joint velocities ')

        if RobotPlotFlags.JOINT_ACC in plotSelection:
            acceleration_fig = plt.figure()
            for k in range(7):
                plt.figure(acceleration_fig.number)
                plt.subplot(7, 1, k + 1)
                plt.plot(self.time_stamp[1:], robot_acceleration[:, k])
                #plt.plot(self.time_stamp, self.uff[:, k], 'g')
                plt.plot(self.time_stamp, self.des_joint_acc[:, k], 'r')
            plt.title(' joint accelerations ')

        if RobotPlotFlags.COMMAND in plotSelection:
            ctrl_torques_fig = plt.figure()
            for k in range(7):
                plt.figure(ctrl_torques_fig.number)
                plt.subplot(7, 1, k + 1)
                plt.plot(self.time_stamp, self.command[:, k])
            plt.title(' control torques (without gravity comp.)')

        if RobotPlotFlags.TORQUES in plotSelection:
            clipped_torques_fig = plt.figure()
            for k in range(7):
                plt.figure(clipped_torques_fig.number)
                plt.subplot(7, 1, k + 1)
                plt.plot(self.time_stamp, self.uff[:, k])
            plt.title(' clipped torques (with gravity comp.)')

        if RobotPlotFlags.GRAVITY in plotSelection:
            grav_terms_fig = plt.figure()
            for k in range(7):
                plt.figure(grav_terms_fig.number)
                plt.subplot(9, 1, k + 1)
                plt.plot(self.time_stamp, self.grav_terms[:, k])
            plt.title(' gravity (+coriolis) ')

        if RobotPlotFlags.LOADS in plotSelection:
            loads_fig = plt.figure()
            for k in range(7):
                plt.figure(loads_fig.number)
                plt.subplot(7, 1,k+1)
                plt.plot(self.time_stamp, self.load[:, k])
            plt.title(' loads - forces in joints')

        if RobotPlotFlags.GRIPPER_WIDTH in plotSelection:
            gripper_fig = plt.figure()
            plt.figure(gripper_fig.number)
            plt.plot(self.time_stamp, self.gripper_width)
            plt.title(' gripper width ')

        if RobotPlotFlags.GRIPPER_FORCE in plotSelection:
            gripper_force_fig = plt.figure()
            plt.figure(gripper_force_fig.number)
            plt.plot(self.time_stamp, self.uff[:, -2:])

            plt.title(' gripper force (without contact forces)')

        if RobotPlotFlags.GRIPPER_POS in plotSelection:
            gripper_pos_fig = plt.figure()
            plt.figure(gripper_pos_fig.number)
            plt.plot(self.time_stamp, self.finger_pos[:, -2:])
            plt.title(' gripper pos ')

        if RobotPlotFlags.CART_POS in plotSelection:
            cart_pos_fig = plt.figure()
            for j in range(3):
                plt.figure(cart_pos_fig.number)
                plt.subplot(3, 1, j + 1)
                plt.plot(self.time_stamp, self.cart_pos[:, j])
                plt.plot(self.time_stamp, self.des_c_pos[:, j], 'r')
            plt.title(' endeffector pos ')

        if RobotPlotFlags.CART_VEL in plotSelection:
            cart_vel_fig = plt.figure()
            for j in range(3):
                plt.figure(cart_vel_fig.number)
                plt.subplot(3, 1, j + 1)
                plt.plot(self.time_stamp, self.cart_vel[:, j])
                plt.plot(self.time_stamp, self.des_c_vel[:, j], 'r')
            plt.title(' endeffector vel ')

        if RobotPlotFlags.ORIENTATION in plotSelection:
            orientation_fig = plt.figure()
            for j in range(4):
                plt.figure(orientation_fig.number)
                plt.subplot(4, 1, j + 1)  # w, x, y, z
                plt.plot(self.time_stamp, self.cart_quat[:, j])
                plt.plot(self.time_stamp, self.des_quat[:, j], 'r')
            plt.title(' endeffector orientation ')

        if RobotPlotFlags.ORIENTATION_VEL in plotSelection:
            orientation_vel_fig = plt.figure()
            for j in range(4):
                plt.figure(orientation_vel_fig.number)
                plt.subplot(4, 1, j + 1)
                plt.plot(self.time_stamp, self.cart_quat_vel[:, j])
                plt.plot(self.time_stamp, self.des_quat_vel[:, j], 'r')
            plt.title(' endeffector angular velocity ')

        if RobotPlotFlags.TIME_STAMPS in plotSelection:
            timeStamp_fig = plt.figure()
            plt.figure(timeStamp_fig.number)
            plt.plot(np.diff(self.time_stamp))
            plt.title('time stamp difference ')
        plt.show()


class ObjectLogger(LoggerBase):
    """
    Logger for the
    - joint positions
    - joint velocities
    - cartesian positions
    - cartesian velocities
    - orientations in w, x, y, z
    - controller commands (without gravity comp)
    - (clipped) commands with gravity
    - gravity compensation terms
    - joint accelerations
    - joint loads

    Resets all logs by calling :func:`startLogging`. Appends current state of each quantity by calling :func:`logData`.
    Stops logging by calling :func:`stopLogging` and plots data by calling :func:`plot`.

    :return: no return value
    """

    def __init__(self, object):
        super().__init__(object)
        """
        Initialization of the quantities to log.
        """

        self.isLogging = False
        self.pos_list = []
        self.vel_list = []
        self.orientation_list = []
        self.orientation_vel_list = []
        self.force_list = []
        self.torque_list = []

        self.pos = []
        self.vel = []
        self.orientation = []
        self.orientation_vel = []
        self.force = []
        self.torque = []


        self.maxTimeSteps = 5000000000000000
        self.object = object

    def startLogging(self):
        """
        Starts logging.

        :return: no return value
        """
        self.pos_list = []
        self.vel_list = []
        self.orientation_list = []
        self.orientation_vel_list = []
        self.force_list = []
        self.torque_list = []

        self.pos = []
        self.vel = []
        self.orientation = []
        self.orientation_vel = []
        self.force = []
        self.torque = []

    def logData(self):
        """
        Appends current state of each logged value.

        :param robot: instance of the robot
        :return: no return value
        """

        # TODO: Fill that in once object class has been created
        self.pos_list.append(self.object.current_pos)
        self.vel_list.append(self.object.current_pos)
        self.orientation_list.append(self.object.current_pos)
        self.orientation_vel_list.append(self.object.current_pos)
        self.force_list.append(self.object.current_pos)
        self.torque_list.append(self.object.current_pos)

        if len(self.pos_list) > self.maxTimeSteps:
            self.pos_list.pop(0)
            self.vel_list.pop(0)
            self.orientation_list.pop(0)
            self.orientation_vel_list.pop(0)
            self.force_list.pop(0)
            self.torque_list.pop(0)

    def stopLogging(self):
        """
        Stops logging.

        :return: No return value
        """

        self.isLogging = False
        if len(self.pos_list) > 0:
            self.pos_list = np.array(self.pos_list)
            self.vel_list = np.array(self.vel_list)

            self.orientation = np.array(self.orientation_list)
            self.orientation_vel = np.array(self.orientation_vel_list)
            self.force = np.array(self.force_list)
            self.torque = np.array(self.torque_list)

    def plot(self, plotSelection = RobotPlotFlags.JOINTS):
        # TODO!!
        raise NotImplementedError