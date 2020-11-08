"""
This module contains the inverse kinematics controller.
"""
import numpy as np
from classic_framework.controllers.Controller import TrackingController


class CartPosController(TrackingController):
    """
    Controller for the cartesian coordinates of the robots end effector.
    """
    def __init__(self):

        TrackingController.__init__(self, dimSetPoint=3)

        self.pgain = np.array([10.0, 10.0, 10.0], dtype=np.float64)
        self.dgain = np.sqrt(self.pgain)

        self.pgain_null = 0.002 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64)
        self.dgain_null = 0.002 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)
        self.desired_c_pos = np.array([0.624, 0, 0.55])
        self.desired_c_vel = np.zeros((3,))
        self.desired_c_acc = np.zeros((3,))

        self.J_reg = 5e-6  # Jacobian regularization constant
        self.W = np.diag([1, 1, 1, 1, 1, 1, 1])

        # Null-space theta configuration
        self.target_th_null = np.array([3.57795216e-09,
                                        1.74532920e-01,
                                        3.30500960e-08,
                                        -8.72664630e-01,
                                        -1.14096181e-07,
                                        1.22173047e+00,
                                        7.85398126e-01])

    def isFinished(self, robot):
        return False

    def initController(self, robot):
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
        xd_d = self.desired_c_pos - robot.current_c_pos
        vd_d = self.desired_c_vel - robot.current_c_vel
        target_c_acc = self.pgain * xd_d + self.dgain * vd_d + self.desired_c_acc

        J = robot.getJacobian()
        J = J[:3, :]
        Jw = J.dot(self.W)

        # J *  W * J' + reg * I
        JwJ_reg = Jw.dot(J.T) + self.J_reg * np.eye(3)

        # Null space movement
        qd_null = self.pgain_null * (self.target_th_null - robot.current_j_pos) + self.dgain_null * (
            -robot.current_j_vel)
        # W J.T (J W J' + reg I)^-1 xd_d + (I - W J.T (J W J' + reg I)^-1 J qd_null

        qd_d = np.linalg.solve(JwJ_reg, target_c_acc - J.dot(qd_null))
        qd_d = self.W.dot(J.transpose()).dot(qd_d) + qd_null

        # qd_d = J.transpose().dot(target_c_acc)
        robot.des_c_pos = self.desired_c_pos
        robot.des_c_vel = self.desired_c_vel

        self.paramsLock.release()
        return qd_d

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
        self.desired_c_pos = desired_pos
        if (desired_vel is not None):
            self.desired_c_vel = desired_vel
        if (desired_acc is not None):
            self.desired_c_acc = desired_acc
        self.paramsLock.release()

    def getCurrentPos(self, robot):
        """
        Getter for the robots current posi
        :param robot:
        :return:
        """
        return robot.current_c_pos


class CartPosQuatController(TrackingController):
    """
    Controller for the cartesian coordinates and the orientation (using quaternions) of the robots end effector.
    """
    def __init__(self):

        TrackingController.__init__(self, dimSetPoint=7)

        self.pgain = np.array([20.0, 20.0, 20.0, 20.0, 20.0, 20.0], dtype=np.float64)
        self.dgain = np.sqrt(self.pgain)

        self.pgain_null = 0.002 * np.array([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0], dtype=np.float64)
        self.dgain_null = 0.002 * np.array([50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0], dtype=np.float64)

        self.desired_c_pos = np.array([0.624, 0, 0.55, 0., 0.984, 0, 0.177])
        self.desired_c_pos[3:] = self.desired_c_pos[3:] / np.linalg.norm(self.desired_c_pos[3:])
        self.desired_c_vel = np.zeros((7,))
        self.desired_c_acc = np.zeros((7,))

        self.J_reg = 1e-2  # Jacobian regularization constant
        self.W = np.diag([1, 1, 1, 1, 1, 1, 1])

        # Null-space theta configuration
        self.target_th_null = np.array([3.57795216e-09,
                                        1.74532920e-01,
                                        3.30500960e-08,
                                        -8.72664630e-01,
                                        -1.14096181e-07,
                                        1.22173047e+00,
                                        7.85398126e-01])

    def isFinished(self, robot):
        return False

    def initController(self, robot):
        return

    def getQuaternionError(self, curr_quat, des_quat):
        """
        Calculates the difference between the current quaternion and the desired quaternion.

        :param curr_quat: current quaternion
        :param des_quat: desired quaternion
        :return: difference between current quaternion and desired quaternion
        """
        quatError = np.zeros((3,))

        quatError[0] = (curr_quat[0] * des_quat[1]
                        - des_quat[0] * curr_quat[1]
                        - curr_quat[3] * des_quat[2]
                        + curr_quat[2] * des_quat[3])

        quatError[1] = (curr_quat[0] * des_quat[2]
                        - des_quat[0] * curr_quat[2]
                        + curr_quat[3] * des_quat[1]
                        - curr_quat[1] * des_quat[3])

        quatError[2] = (curr_quat[0] * des_quat[3]
                        - des_quat[0] * curr_quat[3]
                        - curr_quat[2] * des_quat[1]
                        + curr_quat[1] * des_quat[2])

        return quatError

    def getControl(self, robot):
        self.paramsLock.acquire()
        qd_d = self.desired_c_pos[:3] - robot.current_c_pos
        vd_d = self.desired_c_vel[:3] - robot.current_c_vel

        target_cpos_acc = self.pgain[:3] * qd_d + self.dgain[:3] * vd_d + self.desired_c_acc[:3]
        target_cquat = self.pgain[3:] * self.getQuaternionError(robot.current_c_quat, self.desired_c_pos[3:]) + \
                       self.dgain[3:] * (self.desired_c_vel[3:] - robot.current_c_quat_vel)[1:] + self.desired_c_acc[4:]

        # target_cquat = self.pgain[3:] * self.getQuaternionError(robot.current_c_quat, self.desired_c_pos[3:]) + self.desired_c_acc[4:]

        target_c_acc = np.hstack((target_cpos_acc, target_cquat))

        J = robot.getJacobian()

        Jw = J.dot(self.W)

        # J *  W * J' + reg * I
        JwJ_reg = Jw.dot(J.T) + self.J_reg * np.eye(J.shape[0])

        # Null space movement
        qd_null = self.pgain_null * (self.target_th_null - robot.current_j_pos) + self.dgain_null * (
            -robot.current_j_vel)

        # W J.T (J W J' + reg I)^-1 xd_d + (I - W J.T (J W J' + reg I)^-1 J qd_null
        qd_d = np.linalg.solve(JwJ_reg, target_c_acc - J.dot(qd_null))
        qd_d = self.W.dot(J.transpose()).dot(qd_d) + qd_null

        #print('target_c: ', target_c_acc)
        #print('dq: ', qd_d)
        #J_ = np.zeros(J.shape)
        #J_[:3,:] = J[:3,:]

        #qd_d = J_.transpose().dot(target_c_acc)

        robot.des_c_pos = self.desired_c_pos[:3]
        robot.des_c_vel = self.desired_c_vel[:3]
        robot.des_quat = self.desired_c_pos[3:]
        robot.des_quat_vel = self.desired_c_vel[3:]

        self.paramsLock.release()
        return qd_d

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
        self.desired_c_pos = desired_pos.copy()
        self.desired_c_pos[3:] = self.desired_c_pos[3:] / np.linalg.norm(self.desired_c_pos[3:])
        if desired_vel is not None:
            self.desired_c_vel = desired_vel
        if desired_acc is not None:
            self.desired_c_acc = desired_acc
        self.paramsLock.release()

    def getCurrentPos(self, robot):
        """
        Getter for the robots current positions.

        :param robot: instance of the robot
        :return: current joint position (num_joints, 1)
        """
        current_c_pos = robot.get_end_effector_pos()
        return np.hstack((current_c_pos, robot.current_c_quat))
