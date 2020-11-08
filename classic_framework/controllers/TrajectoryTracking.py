"""
This module uses a controller and a desired position to calculate the trajectory of the robot joints.
"""
from classic_framework.controllers.IKControllers import *
from classic_framework.controllers.Controller import *

from scipy.interpolate import make_interp_spline


class TrajectoryTracker(Controller):
    """
    Base class for controller tracking trajectories. Extends the controller base class.
    """
    def __init__(self, tracker, dt):
        Controller.__init__(self)

        self.startingTime = None
        self.trackingController = tracker
        self.trajectory = None
        self.trajectoryVel = None
        self.trajectoryAcc = None
        self.dt = dt
        self.additionalDuration = 0

    def isFinished(self, robot):
        """
        Checks if the robot is finished performing an action.

        :param robot: instance of the robot
        :return: True if the robot is finished
        """
        timeStep = np.round((robot.time_stamp - self.startingTime) / self.dt)
        #print(timeStep, robot.time_stamp,  self.startingTime)
        return False #timeStep >= self.trajectory.shape[0] + self.additionalDuration / 0.001

    def initController(self, robot, maxDuration):
        """
        Initialization of the controller.

        :param robot: instance of the robot
        :param maxDuration: maximal control duration
        :return: no return value
        """
        robot.receiveState()
        self.startingTime = robot.time_stamp  # Current robot time stamp
        self.duration = maxDuration

    def getControl(self, robot):
        if self.trajectory is None:
            print('Error: Trajectory is empty')

        self.paramsLock.acquire()

        timeStep = np.round((robot.time_stamp - self.startingTime) / self.dt)
        timeStep = int(np.min([timeStep, self.trajectory.shape[0] - 1]))

        desired_pos = self.trajectory[timeStep, :]

        if timeStep < self.trajectory.shape[0] - 1:
            desired_vel = self.trajectoryVel[timeStep, :]
        else:
            desired_vel = np.zeros((self.trajectory.shape[1],))

        if timeStep < self.trajectory.shape[0] - 2:
            desired_acc = self.trajectoryAcc[timeStep, :]
        else:
            desired_acc = np.zeros((self.trajectory.shape[1],))

        self.trackingController.setSetPoint(desired_pos, desired_vel, desired_acc)
        self.paramsLock.release()

        return self.trackingController.getControl(robot)

    def setTrajectory(self, trajectory):
        """
        Set the trajectory from splines.
        :param trajectory: numpy array (num_time_stamps, num_joints)
        :return: no return value
        """
        self.paramsLock.acquire()

        self.trajectory = trajectory
        self.trajectoryVel = np.diff(trajectory, 1, axis=0) / self.dt
        self.trajectoryAcc = np.diff(trajectory, 2, axis=0) / (self.dt ** 2)

        self.paramsLock.release()


class JointTrajectoryTracker(TrajectoryTracker):
    """
    Tracker for trajectory of the robot joints.
    """
    def __init__(self, dt):
        TrajectoryTracker.__init__(self, JointPDController(), dt)


class CartPosTrajectoryTracker(TrajectoryTracker):
    """
    Tracker for the cartesian coordinates of the robot end effector.
    """
    def __init__(self, dt):
        TrajectoryTracker.__init__(self, CartPosController(), dt)


class CartPosQuatTrajectoryTracker(TrajectoryTracker):
    """
    Tracker for the cartesian coordinates and orientation using quaternions of the robot end effector.
    """
    def __init__(self, dt):
        TrajectoryTracker.__init__(self, CartPosQuatController(), dt)


class GotoController(TrajectoryTracker):
    """
    This class sets the robot trajectory with :func:`initController`. The end effector position is set with
    :func:`setDesiredPos`.
    """

    def __init__(self, tracker, dt):
        """
        Initializes the tracker for the robots trajectory and sets the default value for the duration and
        joint positions.

        :param tracker: tracks robot trajectory
        """

        TrajectoryTracker.__init__(self, tracker, dt)

        self.duration = 4.0  # default duration
        self.desiredPosition = np.array([0, 0, 0, - 1.562, 0, 1.914, 0])  # default joint positions

    def initController(self, robot, maxDuration, fingerController = False):         # param called for choosing, if we want to use last point of spline ( if existing ) for planning the new spline
        """
        This method calls :func:`setTrajectory` and and sets the robot trajectory as [num_timesteps x num_joints].
        The number of time stamps is calculated by maxDuration / 1e-3 (i.e. 1kHz sampling).

        :param robot: instance of the robot
        :param maxDuration: sets the number of time stamps
        :param fingerController: controller for the robot finger joints
        :return: no return value
        """
        super().initController(robot, maxDuration)

        time = np.linspace(0, self.duration, int(self.duration / self.dt))  # create time stamp array
        trajectory = np.zeros((time.shape[0], self.trackingController.dimSetPoint))  # create empty trajectory array

        called = robot.smooth_spline

        if called:
            try:
                if self.trajectory is None:
                    print('first time creating spline: using current position as starting position')
                    cur_state = self.trackingController.getCurrentPos(robot)
                else:
                    print('trajectory was already set, using last desired point of last spline as starting position')
                    cur_state = self.getSetPosFromRobot(robot)
            except Exception:
                print('trajectory was already set, using last desired point of last spline as starting position')
                cur_state = self.getSetPosFromRobot(robot)
        else:
            print(' using current position for setting starting position of current spline')
            cur_state = self.trackingController.getCurrentPos(robot)

        for i in range(self.trackingController.dimSetPoint):
            # This creates a b spline with 0 1st and 2nd order derivatives at the boundaries
            l, r = [(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]
            bsplinef = make_interp_spline(x=[0, self.duration],
                                          y=[cur_state[i], self.desiredPosition[i]],
                                          bc_type=(l, r),
                                          k=5)
            trajectory[:, i] = bsplinef(time)

        self.setTrajectory(trajectory)  # sets trajectory with [num_timesteps x num_joints] from splines

    def setDesiredPos(self, desiredPosition):
        """
        Sets the desired positions of the robot joints.

        :param desiredPosition: numpy array with dim [num_joints,]
        :return: no return value
        """
        self.paramsLock.acquire()

        self.desiredPosition = desiredPosition

        self.paramsLock.release()

    def resetTrajectory(self):
        """
        Sets the trajectory object to None (used if we expect discontinuities)
        """
        self.trajectory = None

    def getSetPosFromRobot(self, robot):
        return robot.des_joint_pos


class GotoJointController(GotoController):
    """
    Controller for the robot joints.
    """
    def __init__(self, dt):
        GotoController.__init__(self, JointPDController(), dt)


class GotoCartPosImpedanceController(GotoController):
    """
    Controller for the cartesian coordinates of the robot.
    """
    def __init__(self, dt):
        GotoController.__init__(self, CartPosController(), dt)

    def getSetPosFromRobot(self, robot):
        return robot.des_c_pos

class GotoCartPosQuatImpedanceController(GotoController):
    """
    Controller for the cartesian coordinates and the orientation (using quaternions) of the robot.
    """
    def __init__(self, dt):
        GotoController.__init__(self, CartPosQuatController(), dt)

    def getSetPosFromRobot(self, robot):
        return np.concatenate((robot.des_c_pos, robot.des_quat))

class GotoCartPosQuatPlanningController(GotoController):

    def __init__(self, dt):
        GotoController.__init__(self, JointPDController(), dt)
        self.desiredTaskPosition = np.zeros(7,)
        from classic_framework.interface.FictiveRobot import FictiveRobot
        self.fictive_robot = FictiveRobot(init_j_pos=np.zeros((7,)), dt=self.dt, offset=np.zeros(3))

    def initController(self, robot, maxDuration, fingerController=False):

        self.fictive_robot.init_j_pos = robot.current_j_pos.copy()
        self.fictive_robot.time_stamp = 0

        self.fictive_robot.gotoCartPositionAndQuat(self.desiredTaskPosition[:3], self.desiredTaskPosition[3:],
                                                   duration=maxDuration)

        des_joints = self.fictive_robot.current_j_pos


        data = robot.config.load_yaml('PD_control_gains')
        pgain = np.array(data['pgain'], dtype=np.float64)
        dgain = np.array(data['dgain'], dtype=np.float64)
        self.trackingController.pgain = pgain
        self.trackingController.dgain = dgain

        self.desiredPosition = des_joints
        super().initController(robot, maxDuration, fingerController)

        self.fictive_robot.des_joint_traj = []

    def setDesiredPos(self, desiredTaskPosition):
        """
        Sets the desired positions of the robot joints.

        :param desiredPosition: numpy array with dim [num_joints,]
        :return: no return value
        """
        self.paramsLock.acquire()

        self.desiredTaskPosition = desiredTaskPosition

        self.paramsLock.release()