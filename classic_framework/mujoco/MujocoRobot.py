import mujoco_py
import numpy as np

from classic_framework import RobotBase
from classic_framework.controllers.TrajectoryTracking import GotoCartPosQuatImpedanceController
from classic_framework.controllers.TrajectoryTracking import GotoCartPosQuatPlanningController
from classic_framework.controllers.TrajectoryTracking import GotoJointController
from classic_framework.controllers.TrajectoryTracking import JointTrajectoryTracker
from classic_framework.interface.FictiveRobot import FictiveRobot
from classic_framework.mujoco.mujoco_utils.mujoco_helpers import reset_mocap2body_xpos
from classic_framework.utils.sim_path import sim_framework_path


class MujocoRobot(RobotBase):
    def __init__(self, scene, config_path=None, gravity_comp=True, clip_actions=False, num_DoF=7):
        if config_path is None:
            config_path = sim_framework_path('./classic_framework/controllers/Config/Mujoco/Standard')
        super(MujocoRobot, self).__init__(config_path, num_DoF=num_DoF, dt=scene.dt)

        self.scene = scene
        self.control_mode = self.scene.control.ctrl_name
        self.functions = mujoco_py.functions
        self.sim = scene.sim
        self.model = scene.model
        self.viewer = scene.viewer
        self.render = scene.render

        self.clip_actions = clip_actions
        self.gravity_comp = gravity_comp

        self.gotoJointController = GotoJointController(self.dt)
        self.gotoCartPosController = GotoCartPosQuatPlanningController(self.dt)
        self.gotoCartPosController.fictive_robot.offset = np.array([0., 0., 0.])

        self.gotoCartPosQuatController = GotoCartPosQuatPlanningController(self.dt)
        self.gotoCartPosQuatController.fictive_robot.offset = np.array([0., 0., 0.])

        self.gotoCartPosImpedanceController = GotoCartPosQuatImpedanceController(self.dt)

        self.jointTrajectoryTracker = JointTrajectoryTracker(self.dt)

        # self.mocap_setup = None
        self.reset()

    def reset(self):
        """
        Reset robot and scene in mujoco
        """
        super(MujocoRobot, self).reset_base()
        self.scene.sim.reset()  # resets everything to 0.... Maybe we should use MjSimState for resetting
        self.scene.set_q(joint_pos=self.scene.init_qpos[:7])
        self.scene.sim.forward()
        self.mocap_setup = None
        self.receiveState()

    def getJacobian(self, q=None):
        """
        Getter for the jacobian matrix.

        :return: jacobian matrix
        """
        jac = np.zeros((6, 7))
        if q is not None:
            # if we want to have the jacobian for specific joint constelation, do this, otherwise
            # directly read out the jacobian
            # see: http://www.mujoco.org/forum/index.php?threads/inverse-kinematics.3505/

            # first copy current simulation state
            cur_sim_state = self.sim.get_state()
            self.sim.data.qpos[:7] = q
            self.functions.mj_kinematics(self.model, self.sim.data)
            self.functions.mj_comPos(self.model, self.sim.data)

        jac[:3, :] = self.sim.data.get_body_jacp('tcp').reshape((3, -1))[:, :7]
        jac[3:, :] = self.sim.data.get_body_jacr('tcp').reshape((3, -1))[:, :7]

        if q is not None:
            # we have to reset the simulation to the state from before
            self.sim.set_state(cur_sim_state)
            # NOTE: Test followin workflow:
            # 		get jacobian for current simulation step
            # 		set qpos to a random position
            #	    call forward kinematics and compos
            #	    get new jacobian -> should be different to the one from before
            # 		reset simulation to the state before (with self.sim.set_state
            # 		calculate again jacobian -> should be the same from the state before.
            # 		BUT: IT is not!!! why?
        return jac

    def getForwardKinematics(self, q=None):
        if q is not None:
            # first copy current simulation state
            cur_sim_state = self.sim.get_state()
            self.sim.data.qpos[:7] = q
            self.functions.mj_kinematics(self.model, self.sim.data)
        cart_pos = self.sim.data.get_body_xpos('tcp')
        cart_or = self.sim.data.get_body_xquat('tcp')
        if q is not None:
            # reset simulation back to state from before
            self.sim.set_state(cur_sim_state)
        return cart_pos, cart_or

    def nextStep(self):

        if self.time_stamp != 0:
            self.preprocessCommand(self.command)
            self.sim.data.ctrl[:] = self.uff.copy()

            # execute simulation for one step
            try:
                self.sim.step()
            except Exception:
                print("Simulation step could not be executed")
            if self.render:
                self.viewer.render()
            self.num_calls += 1
            if self.logger.isLogging:
                self.logger.logData()
            self.receiveState()
            self.initCounter = self.initCounter + 1

        self.counter = self.counter + 1
        self.time_stamp += 1 * 0.001

        # self.des_joint_pos = np.zeros((self.num_DoF,)) * np.nan
        # self.des_joint_vel = np.zeros((self.num_DoF,)) * np.nan
        # self.des_joint_acc = np.zeros((self.num_DoF,)) * np.nan

        # self.des_c_pos = np.zeros((3,)) * np.nan
        # self.des_c_vel = np.zeros((3,)) * np.nan

        # self.des_quat = np.zeros((4,)) * np.nan
        # self.des_quat_vel = np.zeros((4,)) * np.nan

    def receiveState(self):

        # joints
        self.current_j_pos = np.array([self.sim.data.get_joint_qpos(name) for name in self.scene.joint_names].copy())
        self.current_j_vel = np.array([self.sim.data.get_joint_qvel(name) for name in self.scene.joint_names].copy())

        # end effector/tip
        self.current_c_pos = self.sim.data.get_body_xpos('tcp').copy()
        self.current_c_vel = self.sim.data.get_body_xvelp('tcp').copy()
        self.current_c_quat = self.sim.data.get_body_xquat('tcp').copy()  # [w, x, y, z]
        self.current_c_quat_vel = np.zeros(4)
        self.current_c_quat_vel[1:] = self.sim.data.get_body_xvelr('tcp').copy()
        self.current_c_quat_vel *= 0.5 * self.current_c_quat

        # fingers
        self.current_fing_pos = [self.sim.data.get_joint_qpos(j_name) for j_name in self.scene.gripper_names]
        self.current_fing_vel = [self.sim.data.get_joint_qvel(j_name) for j_name in self.scene.gripper_names]
        self.gripper_width = self.current_fing_pos[-2] + self.current_fing_pos[-1]

        # self.des_joint_pos = np.zeros((self.num_DoF,)) * np.nan
        # self.des_joint_vel = np.zeros((self.num_DoF,)) * np.nan
        # self.des_joint_acc = np.zeros((self.num_DoF,)) * np.nan

    def get_command_from_inverse_dynamics(self, target_j_acc, mj_calc_inv=False):
        if mj_calc_inv:
            self.sim.data.qacc[:target_j_acc.shape[0]] = target_j_acc
            self.functions.mj_inverse(self.model, self.sim.data)
            return self.sim.data.qfrc_inverse[:9]  # 9 since we have 2 actuations on the fingers
        else:
            return self.sim.data.qfrc_bias[:9]

    def gotoCartPositionAndQuat(self, desiredPos, desiredQuat, use_fictive_robot=True, duration=4.0):
        # init_j_pos = np.array([self.sim.data.get_joint_qpos(name) for name in self.scene.joint_names].copy())
        if self.control_mode == 'ik' or self.control_mode == 'torque':
            # if use_fictive_robot:
            # 	# initial_simulation_state = self.sim.get_state()		# not needed since we are not using mujocofictive robot
            # 	if self.fictive_robot is None:
            # 		# self.fictive_robot = MujocoRobot_Fictive(init_j_pos=init_j_pos, scene=self.scene) # if we do not want to use pinocchios fictive robot
            # 		self.fictive_robot = FictiveRobot(init_j_pos=init_j_pos, dt=self.scene.dt, offset=np.zeros(3))
            # 	else:
            # 		self.fictive_robot.init_j_pos = init_j_pos
            # 		self.fictive_robot.time_stamp = 0
            # 	self.fictive_robot.gotoCartPositionAndQuat(desiredPos, desiredQuat, duration=duration)
            # 	desired_cart_traj = self.fictive_robot.gotoCartPosQuatController.trajectory
            # 	desired_cart_vels = self.fictive_robot.gotoCartPosQuatController.trajectoryVel
            # 	des_joints = self.fictive_robot.current_j_pos
            # 	des_joints_traj = np.array(self.fictive_robot.des_joint_traj)
            # 	self.fictive_robot.des_joint_traj = []
            # 	# reset the simulation to the state from before
            # 	# self.sim.set_state(initial_simulation_state)		# not needed anymore since we are not using mujoco fictive robot
            # 	self.gotoJointPosition(des_joints, duration=duration)
            # 	# self.follow_JointTraj(des_joints_traj)			# we can directly have the joint trajectory from fictive robot
            # 														# but it showed that replanning in joint space gives
            # 														# better results
            #
            # 	if desired_cart_traj is not None:
            # 		if np.isnan(self.logger.des_c_pos_list[-1][0]):
            # 			self.logger.des_c_pos_list[-np.int(duration / self.dt):] = list(desired_cart_traj[:, :3])
            # 			self.logger.des_c_vel_list[-np.int(duration / self.dt):] = list(desired_cart_vels[:, :3])
            #
            # 			self.logger.des_quat_list[-np.int(duration / self.dt):] = list(desired_cart_traj[:, 3::])
            # 			self.logger.des_quat_vel_list[-np.int(duration / self.dt):] = list(desired_cart_vels[:, 3::])
            # 		else:
            # 			self.logger.des_c_pos_list += list(desired_cart_traj[:, :3])
            # 			self.logger.des_c_vel_list += list(desired_cart_vels[:, :3])
            #
            # 			self.logger.des_quat_list += list(desired_cart_traj[:, 3::])
            # 			self.logger.des_quat_vel_list += list(desired_cart_vels[:, 3::])
            # else:
            # 	super(MujocoRobot, self).gotoCartPositionAndQuat(desiredPos, desiredQuat, duration=duration)
            super(MujocoRobot, self).gotoCartPositionAndQuat(desiredPos, desiredQuat, duration=duration)
            # desired_cart_traj = self.gotoCartPosQuatController.fictive_robot.gotoCartPosQuatController.trajectory
            # desired_cart_vels = self.gotoCartPosQuatController.fictive_robot.gotoCartPosQuatController.trajectoryVel


        elif self.control_mode == 'mocap':
            self.mocap_ctrl(desiredPos, desiredQuat, duration)

        else:
            raise ValueError(
                "Error, please choose between <ik> or <mocap> control. <ik> is torque based and allows also"
                "to use gotojointposition")

    def mocap_ctrl(self, desired_pos, orientation, duration=4.):

        # setup
        if not self.mocap_setup:
            if self.sim.model.nmocap > 0 and self.sim.model.eq_data is not None:
                for i in range(self.sim.model.eq_data.shape[0]):
                    if self.sim.model.eq_type[i] == mujoco_py.const.EQ_WELD:
                        self.sim.model.eq_data[i, :] = np.array([0., 0., 0., 1., 0., 0., 0.])

            self.sim.forward()

            # Move end effector into position.
            gripper_target = self.sim.data.get_body_xpos('tcp').copy()
            gripper_rotation = np.array([0., 1., 0., 0.])
            self.sim.data.set_mocap_pos('panda:mocap', gripper_target)
            self.sim.data.set_mocap_quat('panda:mocap', gripper_rotation)
            for _ in range(20):
                self.sim.data.mocap_pos[:] = self.sim.data.get_body_xpos('tcp').copy()
                self.sim.data.mocap_quat[:] = self.sim.data.get_body_xquat('tcp').copy()
                self.sim.step()

            self.mocap_setup = True

        # self.gotoCartPosQuatController.setDesiredPos(np.hstack((desired_pos, orientation)))
        # self.gotoCartPosQuatController.initController(self, maxDuration=duration)
        self.gotoCartPosImpedanceController.setDesiredPos(np.hstack((desired_pos, orientation)))
        self.gotoCartPosImpedanceController.initController(self, maxDuration=duration)

        desired_traj = self.gotoCartPosImpedanceController.trajectory
        for t in range(desired_traj.shape[0]):
            self.receiveState()
            reset_mocap2body_xpos(self.sim)
            self.sim.data.mocap_pos[:] = desired_traj[t, :3].copy()
            self.sim.data.mocap_quat[:] = desired_traj[t, 3:].copy()

            gripper_ctrl = self.fing_ctrl_step()
            self.sim.data.ctrl[:] = gripper_ctrl.copy()

            self.des_c_pos = desired_traj[t, :3].copy()
            self.des_quat = desired_traj[t, 3:].copy()
            if self.logger.isLogging:
                self.logger.logData()
            # Forward the simulation
            self.sim.step()
            self.time_stamp += 1 * 0.001

            # Render the scene
            if self.render:
                self.scene.viewer.render()

    # not tested yet -> test if it is working!
    def beam_to_joint_pos(self, desiredJoints):
        self.scene.set_q(desiredJoints)
        self.receiveState()
        self.gotoJointController.resetTrajectory()
        self.gotoCartPosController.resetTrajectory()
        self.gotoCartPosController.resetTrajectory()

    "Do not delete. Might be usefule "
    # def get_pos_for_panda_hand(self, desiredPos, desiredQuat):
    # 	# first set to inital transformation:
    # 	init_transformation = np.linalg.inv(self.sim.data.get_body_xmat('panda_hand'))
    # 	# init positions
    # 	panda_hand_init = init_transformation @ self.current_c_pos
    # 	site_init = init_transformation @ self.sim.data.get_body_xpos('tcp')
    #
    # 	# transform to target orientation
    # 	target_transf_mat = quat2mat(desiredQuat)
    # 	panda_hand_new = target_transf_mat @ panda_hand_init
    # 	site_new = target_transf_mat @ site_init
    #
    # 	# get delta position to site:
    # 	d_pos = desiredPos - site_new
    #
    # 	# add to panda hand new:
    # 	panda_hand_new += d_pos
    # 	return panda_hand_new


class MujocoRobot_Fictive(FictiveRobot):

    def __init__(self, init_j_pos, scene, config_path=None):
        if config_path is None:
            config_path = sim_framework_path(
                './classic_framework/controllers/Config/Mujoco/FictiveRobot')  # we have set d-part of orientation to 0
        super(MujocoRobot_Fictive, self).__init__(init_j_pos=init_j_pos, config_path=config_path, offset=np.zeros(3),
                                                  dt=scene.dt)
        self.scene = scene
        self.functions = mujoco_py.functions
        self.receiveState()

    def set_joints(self, new_joints):
        self.scene.set_q(new_joints)

    def getForwardKinematics(self, q=None):
        self.functions.mj_kinematics(self.scene.model, self.scene.sim.data)

    def getJacobian(self):
        jac = np.zeros((6, 7))
        jac[:3, :] = self.scene.sim.data.get_body_jacp('tcp').reshape((3, -1))[:, :7]
        jac[3:, :] = self.scene.sim.data.get_body_jacr('tcp').reshape((3, -1))[:, :7]
        return jac

    def extract_new_positions(self):
        self.getForwardKinematics()
        current_c_pos = self.scene.sim.data.get_body_xpos('tcp').copy()
        current_c_vel = self.scene.sim.data.get_body_xvelp('tcp').copy()
        current_c_quat = self.scene.sim.data.get_body_xquat('tcp').copy()
        return current_c_pos, current_c_vel, current_c_quat

    def check_offset_to_pinocchio_model_in_cart_space(self, q):
        import pinocchio
        from classic_framework.utils.geometric_transformation import mat2quat
        obj_urdf = sim_framework_path("./envs/panda_arm_hand_pinocchio.urdf")
        model = pinocchio.buildModelFromUrdf(obj_urdf)
        data = model.createData()

        q_pin = np.zeros(9)
        q_pin[:7] = q
        pinocchio.framesForwardKinematics(model, data, q_pin)
        pinocchio.computeJointJacobians(model, data, q_pin)
        pinocchio.framesForwardKinematics(model, data, q_pin)
        # get orientation
        rotation_matrix = data.oMf[model.getFrameId('panda_grasptarget')].rotation
        quaternions = mat2quat(rotation_matrix)  # [w x y z]
        jac = pinocchio.getFrameJacobian(model, data, model.getFrameId('panda_grasptarget'),
                                         pinocchio.LOCAL_WORLD_ALIGNED)[:, :7]
        print('position offset:',
              self.current_c_pos - np.array(data.oMf[model.getFrameId('panda_grasptarget')].translation))

    def check_orientation_offset(self, q):
        import pinocchio
        from classic_framework.utils.geometric_transformation import mat2quat
        obj_urdf = sim_framework_path("./envs/panda_arm_hand_pinocchio.urdf")
        model = pinocchio.buildModelFromUrdf(obj_urdf)
        data = model.createData()

        q_pin = np.zeros(9)
        q_pin[:7] = q
        pinocchio.framesForwardKinematics(model, data, q_pin)
        pinocchio.computeJointJacobians(model, data, q_pin)
        pinocchio.framesForwardKinematics(model, data, q_pin)
        # get orientation
        rotation_matrix = data.oMf[model.getFrameId('panda_grasptarget')].rotation
        quaternions = mat2quat(rotation_matrix)  # [w x y z]
        jac = pinocchio.getFrameJacobian(model, data, model.getFrameId('panda_grasptarget'),
                                         pinocchio.LOCAL_WORLD_ALIGNED)[:, :7]
        print('orientation offset:',
              self.current_c_quat - quaternions)
        a = 1
