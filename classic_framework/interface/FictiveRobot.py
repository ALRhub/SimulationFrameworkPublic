import numpy as np
import pinocchio

from classic_framework import GotoCartPosImpedanceController
from classic_framework import GotoCartPosQuatImpedanceController
from classic_framework.controllers.TrajectoryTracking import GotoJointController
from classic_framework.controllers.TrajectoryTracking import JointTrajectoryTracker
from classic_framework.interface.Robots import RobotBase
from classic_framework.utils.sim_path import sim_framework_path


class FictiveRobot(RobotBase):

    def __init__(self, init_j_pos, dt, offset=0, config_path=None):
        if config_path is None:
            config_path = sim_framework_path('./classic_framework/controllers/Config/PyBullet/FictiveRobot')
        super().__init__(config_path, dt=dt)

        obj_urdf = sim_framework_path("./envs/panda_arm_hand_pinocchio.urdf")
        self.model = pinocchio.buildModelFromUrdf(obj_urdf)
        self.data = self.model.createData()
        # The offset is [0, 0, 0.88] for pybullet (with the table)
        # The offset is [0, 0, 0] for mujoco (with current scene)
        # NOTE: adjust the offset (z-direction), if you re-place the robot! (Use check offset function of fictive robot)
        self.offset = offset
        self.init_j_pos = init_j_pos
        self.end_effector_frame_id = self.model.getFrameId('panda_grasptarget')
        self.des_joint_traj = []

        self.gotoJointController = GotoJointController(self.dt)
        self.gotoCartPosController = GotoCartPosImpedanceController(self.dt)
        self.gotoCartPosQuatController = GotoCartPosQuatImpedanceController(self.dt)
        self.jointTrajectoryTracker = JointTrajectoryTracker(self.dt)

    def set_joints(self, new_joints):
        # implement here setting the new joints in the simulation
        # Note: After calling this function, we always have to call the fwd_kinematics
        return new_joints

    def getForwardKinematics(self, q=None):
        if q is None:
            q = self.current_j_pos
        if q.shape[0] == 7:
            q_tmp = q.copy()
            q = np.zeros(9)
            q[:7] = q_tmp
        pinocchio.framesForwardKinematics(self.model, self.data, q)

    def getJacobian(self, q=None):
        if q is None:
            q = self.current_j_pos
        if q.shape[0] == 7:
            q_tmp = q.copy()
            q = np.zeros(9)
            q[:7] = q_tmp
        pinocchio.computeJointJacobians(self.model, self.data, q)
        pinocchio.framesForwardKinematics(self.model, self.data, q)
        return pinocchio.getFrameJacobian(self.model, self.data,
                                          self.end_effector_frame_id,
                                          pinocchio.LOCAL_WORLD_ALIGNED)[:, :7]

    def extract_new_positions(self):
        # This function needs to return:
        # 1: end_effector position
        # 2: end_effector velocity
        # 3: end_effector orientation
        current_c_pos = np.array(
            self.data.oMf[self.end_effector_frame_id].translation)
        current_c_pos += self.offset

        # should return 0 always as we do not give torques
        # current_c_vel = np.array(pinocchio.getFrameVelocity(self.model, self.data,
        #                                                     self.model.getFrameId('panda_grasptarget')))
        current_c_vel = np.zeros(3)
        # current_c_vel = (current_c_pos - self.c_pos_old)*self.dt

        rotation_matrix = np.array(
            self.data.oMf[self.end_effector_frame_id].rotation)
        quat_pin = pinocchio.Quaternion(self.data.oMf[
                                            self.end_effector_frame_id].rotation).coeffs()  # [ x, y, z, w]
        current_c_quat = np.zeros(4)
        current_c_quat[1:] = quat_pin[:3]
        current_c_quat[0] = quat_pin[-1]
        return current_c_pos, current_c_vel, current_c_quat

    def receiveState(self):

        if self.time_stamp == 0:
            self.current_j_pos = self.init_j_pos
            self.current_j_vel = np.zeros(7)
            self.set_joints(self.current_j_pos)
            self.getForwardKinematics()
            infos = self.extract_new_positions()
            self.current_c_pos = infos[0]
            self.current_c_vel = infos[1]
            self.current_c_quat = infos[2]
            self.current_c_quat_vel = np.zeros(4)

        self.last_cmd = self.uff
        # self.des_joint_pos = np.zeros((self.num_DoF,)) * np.nan
        # self.des_joint_vel = np.zeros((self.num_DoF,)) * np.nan
        # self.des_joint_acc = np.zeros((self.num_DoF,)) * np.nan

    def nextStep(self):

        if self.time_stamp == 0:
            self.receiveState()

        else:
            """use this function to get your offset: only when changing the robots cartesian position. Adjust only once"""
            # self.check_offset_to_pinocchio_model_in_cart_space(self.current_j_pos)
            # self.check_orientation_offset(self.current_j_pos)
            self.num_calls += 1
            if self.logger.isLogging:
                self.logger.logData()
            self.receiveState()
            self.current_j_pos += self.dt * self.command.copy()
            self.current_j_vel = self.command.copy()
            self.set_joints(self.current_j_pos)
            self.getForwardKinematics()
            state_info = self.extract_new_positions()
            self.current_c_pos = state_info[0]
            self.current_c_vel = state_info[1]
            self.current_c_quat = state_info[2]
            self.current_c_quat_vel = np.zeros(4)
            self.initCounter = self.initCounter + 1

        self.des_joint_traj.append(self.current_j_pos.copy())
        self.counter += 1
        self.time_stamp += self.dt

        # Reset all the joint positions, velocities etc. for next time stamp
        # self.des_joint_pos = np.zeros((self.num_DoF,)) * np.nan
        # self.des_joint_vel = np.zeros((self.num_DoF,)) * np.nan
        # self.des_joint_acc = np.zeros((self.num_DoF,)) * np.nan
        #
        # self.des_c_pos = np.zeros((3,)) * np.nan
        # self.des_c_vel = np.zeros((3,)) * np.nan
        #
        # self.des_quat = np.zeros((4,)) * np.nan
        # self.des_quat_vel = np.zeros((4,)) * np.nan
