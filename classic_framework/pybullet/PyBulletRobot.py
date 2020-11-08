import numpy as np

from classic_framework import RobotBase
from classic_framework.controllers.TrajectoryTracking import GotoCartPosQuatPlanningController
from classic_framework.controllers.TrajectoryTracking import GotoJointController
from classic_framework.controllers.TrajectoryTracking import JointTrajectoryTracker
from classic_framework.interface.FictiveRobot import FictiveRobot
from classic_framework.utils.sim_path import sim_framework_path


class PyBulletRobot(RobotBase):

    def __init__(self, pybullet, scene, config_path=None, gravity_comp=True, pos_ctrl=False, clip_actions=False,
                 num_DoF=7, publisher=None):
        if config_path is None:
            config_path = sim_framework_path('./classic_framework/controllers/Config/PyBullet/Standard')
        super(PyBulletRobot, self).__init__(config_path, num_DoF=num_DoF, dt=scene.dt)

        self.pybullet = pybullet
        self.scene = scene
        self.robot_id, self.robot_id_ik, self.robotEndEffectorIndex = scene.load_panda_to_scene()
        self.pybullet.setTimeStep(self.dt)
        self.jointIndices_with_fingers = [1, 2, 3, 4, 5, 6, 7, 10, 11]
        self.pybullet.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                                                jointIndices=[10, 11],
                                                controlMode=self.pybullet.VELOCITY_CONTROL,
                                                forces=[0., 0.])
        self.jointIndices = [1, 2, 3, 4, 5, 6, 7]
        # enable JointForceTorqueSensor for joints         # maybe enable joint force sensor for gripper here later!!!
        [self.pybullet.enableJointForceTorqueSensor(bodyUniqueId=self.robot_id,
                                                    jointIndex=jointIndex) for jointIndex in
         self.jointIndices_with_fingers]

        self.gravity_comp = gravity_comp
        self.pos_ctrl = pos_ctrl
        if not pos_ctrl:
            self.scene.disable_robot_vel_ctrl(self.robot_id)
        else:
            self.scene.enable_robot_vel_ctrl(self.robot_id, max_forces=self.torque_limit)
        self.clip_actions = clip_actions

        self.gotoJointController = GotoJointController(self.dt)
        self.gotoCartPosController = GotoCartPosQuatPlanningController(self.dt)
        self.gotoCartPosController.fictive_robot.offset = np.array([0., 0., 0.88])

        self.gotoCartPosQuatController = GotoCartPosQuatPlanningController(self.dt)
        self.gotoCartPosQuatController.fictive_robot.offset = np.array([0., 0., 0.88])

        self.jointTrajectoryTracker = JointTrajectoryTracker(self.dt)

        self.receiveState()
        self.publisher = publisher

    def reset(self):
        # super().reset()
        # self.scene --> how to reset scene in PyBullet ?
        # TODO: reset pybullet scene and robot
        raise NotImplementedError

    def get_qdq_J(self, robot_id=None, client_id=None):
        """
        This method calculates the joint positions, the joint velocities and the Jacobian.
        Note the position and the velocity of the fingers is not included here.

        :return: q: joint positions
                dq: joint velocities
                 J: jacobian matrix (6x7)
        """
        if client_id is None:
            client_id = self.scene.physics_client_id
        if robot_id is None:
            robot_id = self.robot_id

        qdq_matrix = np.array([np.array(
            self.pybullet.getJointState(bodyUniqueId=robot_id, jointIndex=jointIndex, physicsClientId=client_id)[:2])
            for jointIndex in np.arange(1, 8)])
        q = qdq_matrix[:, 0]
        dq = qdq_matrix[:, 1]

        # jac_t, jac_r = self.pybullet.calculateJacobian(robot_id, self.robotEndEffectorIndex,
        # 											   [0., 0., 0.1034], list(q) + [0.] * 2, [0.] * 9, [0.] * 9,
        # 											   physicsClientId=client_id)
        jac_t, jac_r = self.pybullet.calculateJacobian(robot_id, self.robotEndEffectorIndex,
                                                       [0., 0., 0.0], list(q) + [0.] * 2, [0.] * 9, [0.] * 9,
                                                       physicsClientId=client_id)

        J = np.concatenate((np.array(jac_t)[:, :7], np.array(jac_r)[:, :7]), axis=0)
        return np.array(q), np.array(dq), J

    # def test_pinocchio(self, q):
    # 	import pinocchio
    # 	from classic_framework.utils.geometric_transformation import mat2quat
    # 	obj_urdf = sim_framework_path("./envs/panda_arm_hand_pinocchio.urdf")
    # 	model = pinocchio.buildModelFromUrdf(obj_urdf)
    # 	data = model.createData()
    #
    # 	q_pin = np.zeros(9)
    # 	q_pin[:7] = q
    # 	pinocchio.framesForwardKinematics(model, data, q_pin)
    # 	pinocchio.computeJointJacobians(model, data, q_pin)
    # 	pinocchio.framesForwardKinematics(model, data, q_pin)
    # 	# get orientation
    # 	rotation_matrix = data.oMf[model.getFrameId('panda_grasptarget')].rotation
    # 	quaternions = mat2quat(rotation_matrix) # [w x y z]
    # 	jac = pinocchio.getFrameJacobian(model, data, model.getFrameId('panda_grasptarget'), pinocchio.LOCAL_WORLD_ALIGNED)[:, :7]
    # 	print('position offset:', self.current_c_pos - data.oMf[model.getFrameId('panda_grasptarget')])

    def get_end_effector_pos(self):
        return self.get_x()[0]

    def get_qdq_fingers(self, robot_id=None, client_id=None):
        """
        This method returns the position and the velocities of the fingers.

        :return: fing_pos: 2x1 position of both fingers as np array
                 fing_vel: 2x1 velocity of both fingers as np array
        """
        if robot_id is None:
            robot_id = self.robot_id
        if client_id is None:
            client_id = self.scene.physics_client_id
        f1_info = self.pybullet.getJointState(bodyUniqueId=robot_id, jointIndex=10, physicsClientId=client_id)
        f2_info = self.pybullet.getJointState(bodyUniqueId=robot_id, jointIndex=11, physicsClientId=client_id)

        fing_pos = np.array([f1_info[0], f2_info[0]])
        fing_vel = np.array([f1_info[1], f2_info[1]])
        return fing_pos, fing_vel

    def get_qdq_joints_fingers(self, robot_id=None, client_id=None):
        """
        This method returns position and velocity of the joints and the fingers combined in one array.

        :return: joint and finger positions as np array (9x1)
                 joint and finger velocities as np array (9x1)
        """
        if robot_id is None:
            robot_id = self.robot_id
        if client_id is None:
            client_id = self.scene.physics_client_id
        qdq_matrix = np.array([np.array(
            self.pybullet.getJointState(bodyUniqueId=robot_id, jointIndex=jointIndex, physicsClientId=client_id)[:2])
            for jointIndex in np.arange(1, 8)])
        q = qdq_matrix[:, 0]
        dq = qdq_matrix[:, 1]
        q = list(q)
        dq = list(dq)
        q_dq_finger = self.get_qdq_fingers()
        q.append(q_dq_finger[0][0])
        q.append(q_dq_finger[0][1])
        dq.append(q_dq_finger[1][0])
        dq.append(q_dq_finger[1][1])
        return np.array(q), np.array(dq)

    def get_joint_reaction_forces(self, robot_id=None, client_id=None):
        """
        Callback to PyBullets `getJointState` to calculate the joint reaction forces.

        :param robot_id: robot ID returned by calling `loadURDF`
        :param client_id: ID of the physics client
        :return: joint reaction forces (num joints, ) with ||Fx, Fy, Fz|| for each joint
        """
        if robot_id is None:
            robot_id = self.robot_id
        if client_id is None:
            client_id = self.scene.physics_client_id
        forces = np.zeros(self.num_DoF)
        for joint in self.jointIndices:
            infs = self.pybullet.getJointState(bodyUniqueId=robot_id, jointIndex=joint, physicsClientId=client_id)[2]
            forces[joint - 1] = np.linalg.norm(np.array(infs[0:3]))
        return forces

    def get_x(self, robot_id=None, client_id=None):
        """
        This method returns the cartesian world position, the cartesian velocity and the quaternion
        orientation of the end effector by calling pyBullets `getLinkState`

        :return: robot_x: cartesian world coordinates of end effector
                 robot_dx_dt: cartesian velocity of end effector
                 robot_quat: quaternion end effector orientation
        """
        if robot_id is None:
            robot_id = self.robot_id
        if client_id is None:
            client_id = self.scene.physics_client_id

        # link_infos[0]: Cartesian position of center of mass
        # link_infos[1]: Cartesian orientation of center of mass, in quaternion [x,y,z,w]
        # link_infos[2]: local position offset of inertial frame (center of mass) expressed in the URDF link frame
        # link_infos[3]: local orientation offset of the inertial frame expressed in URDF link frame.
        # link_infos[4]: world position of the URDF link frame
        # link_infos[5]: world orientation of the URDF link frame
        # link_infos[6]: Cartesian world velocity. Only returned if computeLinkVelocity non-zero.

        link_infos = self.pybullet.getLinkState(robot_id,
                                                linkIndex=self.robotEndEffectorIndex,
                                                computeLinkVelocity=1,  # Cartesian world velocity will be returned
                                                physicsClientId=client_id)
        robot_x = np.array(link_infos[4])
        robot_dx_dt = np.array(link_infos[6])
        robot_quat = np.array(link_infos[5])  # quaternion: [x,y,z,w]
        return robot_x, robot_dx_dt, robot_quat

    def get_command_from_inverse_dynamics(self, target_j_acc, mj_calc_inv=False, robot_id=None, client_id=None):
        """
        This method uses the calculation of the inverse Dynamics method of pybullet. Note, that all parameters have to
        be in list format. Otherwise and segmentation get_error is returned.
        Notes on the calculateInverseDynamics function:
            The calculateInverseDynamics function NEEDS all degrees of freedom, which includes also the fingers, since
            they are marked as prismatic joints in the URDF. Only 'fixed joints and the base joint' can be skipped and
            need not to be included in the position and the velocity vectors

        :param q: joint positions and finger positions (have to be given)
        :param client_id: client id of the simulation
        :param robot_id: robot id in the scene
        :param dq: joint velocities and finger velocities
        :param desired_acceleration: The desired acceleration of each degree of freedom as list
        :return: torques for each degree of freedom (9) to achieve the desired acceleration
        """
        if robot_id is None:
            robot_id = self.robot_id
        if client_id is None:
            client_id = self.scene.physics_client_id
        q, dq = self.get_qdq_joints_fingers()
        torques = self.pybullet.calculateInverseDynamics(bodyUniqueId=robot_id, objPositions=list(q),
                                                         objVelocities=list(dq),
                                                         objAccelerations=list(target_j_acc),
                                                         physicsClientId=client_id)
        return np.array(torques)

    def get_invKinematics(self, targetPosition, targetOrientation, robot_id=None, client_id=None):
        if robot_id is None:
            robot_id = self.robot_id
        if client_id is None:
            client_id = self.scene.physics_client_id
        des_orientation = np.zeros(4)
        des_orientation[:3] = targetOrientation[1::]
        des_orientation[-1] = targetOrientation[0]
        des_joints = self.pybullet.calculateInverseKinematics(bodyUniqueId=robot_id,
                                                              endEffectorLinkIndex=self.robotEndEffectorIndex,
                                                              targetPosition=list(targetPosition),
                                                              targetOrientation=list(des_orientation),
                                                              residualThreshold=1e-4, maxNumIterations=4000,
                                                              physicsClientId=client_id)
        return np.array(des_joints)[:7]

    def getJacobian(self):
        """
        Callback to :func:`get_qdq_J` for getting the jacobian matrix.

        :return: numpy array; jacobian matrix
        """
        _, __, J = self.get_qdq_J()
        return J

    def receiveState(self):
        """
        Receives the current state i.e.
        - joint positions, joint velocities by calling `get_qdq_joints_fingers`
        - cartesian coords, velocity and and orientation of the end-effector by calling `get_x()`
        - joint forces by calling `get_joint_reaction_forces`
        - gripper width by calling `getJointState` with finger joint IDs

        --------------------------------------------------------------------------------------------------------------
        Note: PyBullet's quaternion information is always given as [x, y, z, w]
              SL uses the notation: [w, x, y, z]
              We therefore have to do a quick reorganization
              Note that the orientation is also logged in [w, x, y, z]
        --------------------------------------------------------------------------------------------------------------

        :return: no return value
        """

        states = self.get_qdq_joints_fingers()
        current_j_pos = states[0][:7]  # joint positions
        current_j_vel = states[1][:7]  # joint velocities
        self.current_j_vel = current_j_vel
        self.current_j_pos = current_j_pos

        self.current_fing_pos = states[0][7:9]  # finger positions
        self.current_fing_vel = states[1][7:9]  # finger velocities

        cart_infos = self.get_x()
        self.current_c_pos = cart_infos[0]
        self.current_c_vel = cart_infos[1]

        c_quat = np.zeros(4)
        c_quat[1::] = cart_infos[2][:3]
        c_quat[0] = cart_infos[2][-1]
        self.current_c_quat = c_quat  # [w , x, y, z]
        self.current_c_quat_vel = np.zeros(4)
        self.current_c_quat_vel[1:] = self.current_c_vel
        self.current_c_quat_vel *= 0.5 * self.current_c_quat
        self.last_c_pos = self.current_c_pos.copy()

        self.current_load = self.get_joint_reaction_forces()

        # calculate width of the fingers:
        # I don't know if this is correct! Check later!
        self.gripper_width = np.abs(self.pybullet.getJointState(bodyUniqueId=self.robot_id, jointIndex=10)[0] -
                                    self.pybullet.getJointState(bodyUniqueId=self.robot_id, jointIndex=11)[0])
        self.last_cmd = self.uff

    # self.des_joint_pos = np.zeros((self.num_DoF,)) * np.nan
    # self.des_joint_vel = np.zeros((self.num_DoF,)) * np.nan
    # self.des_joint_acc = np.zeros((self.num_DoF,)) * np.nan

    def nextStep(self, target_position=None):
        """
        Executes the simulation for one time stamp, i.e. calling PyBullet's stepSimulation() which will perform all
        the actions in a single forward dynamics simulation step such as collision detection, constraint solving and
        integration.

        :return: no return value
        """
        if self.time_stamp != 0:
            self.preprocessCommand(self.command)
            # set the torques in pybullet
            if self.pos_ctrl:
                max_forces = self.torque_limit
                posGains = np.ones(7) * 0.0015
                posGains[3] *= 10
                posGains[5] *= 10
                # posGains = np.array([12, 12, 12, 12, 50, 30, 10])
                # self.pybullet.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                #                                         jointIndices=self.jointIndices_with_fingers[:-2],
                #                                         controlMode=self.pybullet.POSITION_CONTROL,
                #                                         targetPositions=target_position,
                #                                         targetVelocities=[0, 0, 0, 0, 0, 0, 0],
                #                                         # forces=max_forces, positionGains=np.ones(7)*5,
                #                                         forces=max_forces,
                #                                         positionGains=posGains,
                #                                         # forces=max_forces, positionGains=np.ones(7)*0.005,
                #                                         velocityGains=np.ones(7) * 1)
                self.pybullet.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                                                        jointIndices=self.jointIndices_with_fingers[:-2],
                                                        controlMode=self.pybullet.POSITION_CONTROL,
                                                        targetPositions=target_position)
                finger_commands = self.fing_ctrl_step()
                self.pybullet.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                                                        jointIndices=[10, 11],
                                                        controlMode=self.pybullet.TORQUE_CONTROL,
                                                        forces=finger_commands)
            else:
                self.pybullet.setJointMotorControlArray(bodyUniqueId=self.robot_id,
                                                        jointIndices=self.jointIndices_with_fingers,
                                                        controlMode=self.pybullet.TORQUE_CONTROL,
                                                        forces=self.uff.copy())

            self.pybullet.stepSimulation()  # execute simulation for one step (moves the robot)
            self.num_calls += 1
            if self.logger.isLogging:
                self.logger.logData()
            self.receiveState()
            self.initCounter = self.initCounter + 1

        self.counter = self.counter + 1
        self.time_stamp += 1 * self.dt

        if self.publisher is not None:
            self.publisher.publish_at_freq(self.scene, self.time_stamp)

    # self.des_joint_pos = np.zeros((self.num_DoF,)) * np.nan
    # self.des_joint_vel = np.zeros((self.num_DoF,)) * np.nan
    # self.des_joint_acc = np.zeros((self.num_DoF,)) * np.nan
    #
    # self.des_c_pos = np.zeros((3,)) * np.nan
    # self.des_c_vel = np.zeros((3,)) * np.nan
    #
    # self.des_quat = np.zeros((4,)) * np.nan
    # self.des_quat_vel = np.zeros((4,)) * np.nan

    def beam_to_cart_pos_and_quat(self, desiredPos, desiredQuat):
        des_joints = self.get_invKinematics(desiredPos, desiredQuat)
        self.beam_to_joint_pos(des_joints)

    def beam_to_joint_pos(self, desiredJoints):
        self.scene.set_q(desiredJoints, robot_id=self.robot_id)
        self.receiveState()
        self.gotoJointController.resetTrajectory()
        self.gotoCartPosController.resetTrajectory()
        self.gotoCartPosController.resetTrajectory()

    # we need a use_fictive flag here to make sure that it is also possible to use the inverse Kinematics of pybullet
    # for moving the robot. -> if use_fictive = False the inv kinematics of pybullet will be used
    def gotoCartPositionAndQuat(self, desiredPos, desiredQuat, use_fictive=True, duration=4.0):
        desired_cart_traj = None
        desired_cart_vels = None
        init_j_pos, _, __ = self.get_qdq_J()

        if use_fictive:
            super(PyBulletRobot, self).gotoCartPositionAndQuat(desiredPos, desiredQuat)
        # desired_cart_traj = self.gotoCartPosQuatController.fictive_robot.gotoCartPosQuatController.trajectory
        # desired_cart_vels = self.gotoCartPosQuatController.fictive_robot.gotoCartPosQuatController.trajectoryVel
        else:
            des_joints = self.get_invKinematics(desiredPos, desiredQuat)
            if not self.pos_ctrl:
                self.gotoJointPosition(des_joints, duration=duration)
            else:
                # first let the fingers adjust (because in pos ctrl the robot moves to fast such that fingers are too slow
                # to react)
                for it_fing_adj in range(np.int((duration / self.dt) / 10)):
                    self.nextStep(self.current_j_pos)
                # next let the robot move
                for it in range(np.int(duration / self.dt)):
                    self.nextStep(des_joints)
                    self.des_joint_pos = des_joints

        if desired_cart_traj is not None and not self.pos_ctrl:
            if np.isnan(self.logger.des_c_pos_list[-1][0]):
                self.logger.des_c_pos_list[-np.int(duration / self.dt):] = list(desired_cart_traj[:, :3])
                self.logger.des_c_vel_list[-np.int(duration / self.dt):] = list(desired_cart_vels[:, :3])

                self.logger.des_quat_list[-np.int(duration / self.dt):] = list(desired_cart_traj[:, 3::])
                self.logger.des_quat_vel_list[-np.int(duration / self.dt):] = list(desired_cart_vels[:, 3::])
            else:
                self.logger.des_c_pos_list += list(desired_cart_traj[:, :3])
                self.logger.des_c_vel_list += list(desired_cart_vels[:, :3])

                self.logger.des_quat_list += list(desired_cart_traj[:, 3::])
                self.logger.des_quat_vel_list += list(desired_cart_vels[:, 3::])


class PyBulletRobot_Fictive(FictiveRobot):

    def __init__(self, init_j_pos, pybullet, robotEndEffectorIndex, scene, ik_robot_id, config_path=None):
        if config_path is None:
            move_dirs_up = 2
            config_path = sim_framework_path('./classic_framework/controllers/Config/PyBullet/FictiveRobot')
        super().__init__(init_j_pos, config_path=config_path, offset=np.array([0., 0., 0.88]), dt=scene.dt)

        self.scene = scene
        self.pybullet = pybullet
        self.robotEndEffectorIndex = robotEndEffectorIndex
        self.ik_robot_id = ik_robot_id
        self.ik_client_id = self.scene.ik_client_id
        self.receiveState()

    def set_joints(self, new_joints):
        self.scene.set_q(list(new_joints), robot_id=self.ik_robot_id, physicsClientId=self.ik_client_id)

    def getForwardKinematics(self, q=None):
        return self.pybullet.getLinkState(bodyUniqueId=self.ik_robot_id, linkIndex=self.robotEndEffectorIndex,
                                          computeLinkVelocity=1, computeForwardKinematics=1,
                                          physicsClientId=self.ik_client_id)

    def getJacobian(self):
        q = self.current_j_pos
        jac_t, jac_r = self.pybullet.calculateJacobian(self.ik_robot_id, self.robotEndEffectorIndex,
                                                       [0., 0., 0.0], list(q) + [0.] * 2, [0.] * 9, [0.] * 9,
                                                       physicsClientId=self.ik_client_id)

        J = np.concatenate((np.array(jac_t)[:, :7], np.array(jac_r)[:, :7]), axis=0)
        return J

    def extract_new_positions(self):
        infos = self.getForwardKinematics()
        current_c_pos = np.array(infos[4])
        current_c_vel = np.array(infos[6])
        robot_quat = np.array(infos[5])
        c_quat = np.zeros(4)
        c_quat[1::] = robot_quat[:3]
        c_quat[0] = robot_quat[-1]
        return current_c_pos, current_c_vel, c_quat

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
