import numpy as np
import threading


class Controller:
    """
    Controller base class.
    """
    def __init__(self):
        self.paramsLock = threading.Lock()

    def isFinished(self, robot):
        return False

    def initController(self, robot, maxDuration):
        return

    def getControl(self, robot):
        return 0

    def executeController(self, robot, maxDuration=10):
        """
        Runs the simulation until the position is reached or the maximum duration is exceeded.

        :param robot: instance of the robot
        :param maxDuration:
        :return: no return value
        """
        robot.nextStep()
        self.initController(robot, maxDuration)
        startTime = robot.time_stamp

        while (not self.isFinished(robot)) and (robot.time_stamp - startTime < maxDuration):

            controlAction = self.getControl(robot)  # dim: (num joints,)

            robot.command = controlAction
            robot.nextStep()


class TrackingController(Controller):
    """
    Base class for `JointPDController`, `ZeroTorqueController`. Extends `Controller` class.
    """
    def __init__(self, dimSetPoint):
        Controller.__init__(self)
        self.dimSetPoint = dimSetPoint

    def setSetPoint(self, desired_pos, desired_vel=None, desired_acc=None):
        return

    def getCurrentPos(self, robot):
        """
        Getter for the current joint positions.

        :param robot: instance of the robot
        :return: current joint position (num_joints, 1)
        """
        return robot.current_j_pos


class JointPDController(TrackingController):
    """
    PD Controller for controlling robot joints. Extends `TrackingController` class.
    """
    def __init__(self):
        TrackingController.__init__(self, dimSetPoint=7)

        self.pgain = 0.2 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64)
        self.dgain = 0.2 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)
        #
        self.desired_joint_pos = np.array([0, 0, 0, - 1.562, 0, 1.914, 0])
        self.desired_joint_vel = np.zeros((7,))
        self.desired_joint_acc = np.zeros((7,))

    def isFinished(self, robot):
        return False

    def initController(self, robot, maxDuration):
        return

    def getControl(self, robot):
        """
        Calculates the robot joint acceleration based on
        - the current joint velocity
        - the current joint positions

        :param robot: instance of the robot
        :return: target joint acceleration (num_joints, )
        """
        self.paramsLock.acquire()
        qd_d = self.desired_joint_pos - robot.current_j_pos
        vd_d = self.desired_joint_vel - robot.current_j_vel
        target_j_acc = self.pgain * qd_d + self.dgain * vd_d + self.desired_joint_acc  # original

        robot.des_joint_pos = self.desired_joint_pos.copy()
        robot.des_joint_vel = self.desired_joint_vel.copy()
        robot.des_joint_acc = self.desired_joint_acc.copy()

        self.paramsLock.release()
        return target_j_acc

    def setGains(self, pGain, dGain):
        """
        Setter for the gains of the PD Controller.

        :param pGain: p gain
        :param dGain: d gain
        :return: no return value
        """
        self.paramsLock.acquire()
        self.pgain = pGain
        self.dgain = dGain
        self.paramsLock.release()

    def setSetPoint(self, desired_pos, desired_vel = None, desired_acc=None):
        """
        Sets the desired position, velocity and acceleration of the joints.

        :param desired_pos: desired position (num_joints,)
        :param desired_vel: desired velocity (num_joints,)
        :param desired_acc: desired acceleration (num_joints,)
        :return: no return value
        """
        self.paramsLock.acquire()
        self.desired_joint_pos = desired_pos
        if desired_vel is not None:
            self.desired_joint_vel = desired_vel
        if desired_acc is not None:
            self.desired_joint_acc = desired_acc
        self.paramsLock.release()

    def getCurrentPos(self, robot):
        """
        Getter for the current joint positions.

        :param robot: instance of the robot
        :return: current joint position (num_joints, 1)
        """
        return robot.current_j_pos


class zeroTorqueController(TrackingController):
    """
    Zero torque PD-Controller. Extends `TrackingController` class.
    """
    def __init__(self):
        TrackingController.__init__(self, dimSetPoint=7)

        self.pgain = np.zeros((7,))
        self.dgain = np.zeros((7,))

        self.desired_joint_pos = np.zeros((7,))
        self.desired_joint_vel = np.zeros((7,))
        self.desired_joint_acc = np.zeros((7,))

    def isFinished(self, robot):
        return False

    def initController(self, robot, maxDuration):
        return

    def getControl(self, robot):
        """
        Calculates the robot joint acceleration based on
        - the current joint velocity
        - the current joint positions

        :param robot: instance of the robot
        :return: target joint acceleration (num_joints, )
        """
        self.paramsLock.acquire()
        target_j_acc = np.zeros((7,))

        robot.des_joint_pos = self.desired_joint_pos.copy()
        robot.des_joint_vel = self.desired_joint_vel.copy()

        self.paramsLock.release()
        return target_j_acc

    def setGains(self, pGain, dGain):
        """
        Setter for the gains of the PD Controller.

        :param pGain: p gain
        :param dGain: d gain
        :return: no return value
        """
        self.paramsLock.acquire()
        self.pgain = pGain
        self.dgain = dGain
        self.paramsLock.release()

    def setSetPoint(self, desired_pos, desired_vel=None, desired_acc=None):
        """
        Sets the desired position, velocity and acceleration of the joints.

        :param desired_pos: desired position (num_joints,)
        :param desired_vel: desired velocity (num_joints,)
        :param desired_acc: desired acceleration (num_joints,)
        :return: no return value
        """
        self.paramsLock.acquire()
        self.desired_joint_pos = desired_pos
        if (desired_vel is not None):
            self.desired_joint_vel = desired_vel
        if (desired_acc is not None):
            self.desired_joint_acc = desired_acc
        self.paramsLock.release()

    def getCurrentPos(self, robot):
        """
        Getter for the current joint positions.

        :param robot: instance of the robot
        :return: current joint position (num_joints, 1)
        """
        return robot.current_j_pos


class DampingController(Controller):
    """
    Damping (D) Controller.
    """
    def __init__(self):
        Controller.__init__(self)
        self.dgain = 0.1 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)

    def getControl(self, robot):
        """
        Calculates the robot joint acceleration based on
        - the current joint velocity

        :param robot: instance of the robot
        :return: target joint acceleration (num_joints, )
        """
        self.paramsLock.acquire()
        target_j_acc = - self.dgain * robot.current_j_vel
        self.paramsLock.release()
        return target_j_acc

    def setGains(self, dGain):
        """
        Setter for the gains of the Damping (D) Controller.

        :param dGain: d gain
        :return: no return value
        """

        self.paramsLock.acquire()
        self.dgain = dGain
        self.paramsLock.release()
