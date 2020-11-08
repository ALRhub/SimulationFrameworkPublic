"""
This module contains the :class:`Scene` class which is used to setup a scene for a robot simulation using PyBullet.
"""
import numpy as np
import pybullet as p
from pybullet_utils.bullet_client import BulletClient

from classic_framework import Scene
from classic_framework.pybullet.PyBullet_Camera import InHandCamera, CageCamera
from classic_framework.utils.sim_path import sim_framework_path


class PyBulletScene(Scene):
    """
    This class allows to build a scene for the robot simulation. The standard scene is a model of the Panda robot on a
    table. The returned ids of the assets are saved as an attribute to the object of the Panda_Robot.
    The .urdf files which contain the scene assets (e.g. cubes etc.) are saved in the 'envs' folder of the project.
    """

    def __init__(self, object_list=None, dt=0.001, render=True, realtime=False):
        super(PyBulletScene, self).__init__(object_list=object_list, dt=dt, render=render)
        """
        Initialization of the physics client and cameras (in-hand and cage cam). Calls :func:`setup_scene`.

        :param realtime:  Enable or disable real time simulation (using the real time clock, RTC) in the physics server.
        """
        self.physics_client_id = None
        self.ik_client_id = None
        self.robot_physics_client_id = None
        self.robot_ik_client_id = None
        self.setup_scene()
        self.realtime = realtime
        self.inhand_cam = InHandCamera()
        self.cage_cam = CageCamera()
        self.obj_name2id = {}
        if self.realtime:
            p.setRealTimeSimulation(1)

    def setup_scene(self):
        """
        This function creates a scene.

        :return: no return value
        """

        print("Scene setup")

        # Connect with simulator
        if self.render:
            self.physics_client = BulletClient(p.GUI)  # o r p.DIRECT for non-graphical version
        else:
            self.physics_client = BulletClient(p.DIRECT)  # o r p.DIRECT for non-graphical version

        self.physics_client_id = self.physics_client._client
        self.ik_client = BulletClient(connection_mode=p.DIRECT)
        self.ik_client_id = self.ik_client._client
        p.setPhysicsEngineParameter(enableFileCaching=0)
        # --------------------------------------------------------------------------------------------------------------
        # # Load scene
        # --------------------------------------------------------------------------------------------------------------
        # module_path = path.dirname(path.abspath(__file__))
        # os.path.abspath(os.curdir)
        # os.chdir("..")
        # module_path = os.path.abspath(os.curdir)
        # os.chdir(os.getcwd() + os.sep + os.pardir)  # moves to parent directory
        # module_path = os.getcwd()
        if self.object_list is not None:
            for obj in self.object_list:
                self.load_object_to_scene(path_to_urdf=obj.data_dir + "/" + obj.urdf_name,
                                          orientation=obj.orientation,
                                          position=obj.position,
                                          id_name=obj.object_name)

        self.scene_id = p.loadURDF(sim_framework_path("./envs/plane/plane.urdf"),
                                   physicsClientId=self.physics_client_id)
        self.scene_id_ik = p.loadURDF(sim_framework_path("./envs/plane/plane.urdf"), physicsClientId=self.ik_client_id)

        # load table
        table_urdf = sim_framework_path("./envs/table/table.urdf")
        table_start_position = [0.35, 0.0, 0.0]
        table_start_orientation = [0.0, 0.0, 0.0]
        table_start_orientation_quat = p.getQuaternionFromEuler(table_start_orientation)
        self.table_id = p.loadURDF(table_urdf,
                                   table_start_position,
                                   table_start_orientation_quat,
                                   flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE,
                                   physicsClientId=self.physics_client_id)

        self.table_id_ik = p.loadURDF(table_urdf,
                                      table_start_position,
                                      table_start_orientation_quat,
                                      flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE,
                                      physicsClientId=self.ik_client_id)

        p.setGravity(0, 0, -9.81)

    def load_panda_to_scene(self, physics_robot=True, orientation=None, position=None, id_name=None, path_to_urdf=None,
                            init_q=None):
        """
        This function loads another panda robot to the simulation environment. If loading the object fails, the program is stopped
        and an appropriate get_error message is returned to user.
        :param physics_robot: number of active robots in physics client
        :param path_to_urdf: the whole path of the location of the urdf file to be load as string. If none use default
        :param orientation: orientation of the robot. Can be either euler angles, or quaternions. NOTE: quaternions
                            notation: [x, y, z, w] !
        :param position: cartesian world position to place the robot
        :param id_name: string valued name of the robot. This name can then be called as self.'name'_id to get the
                        id number of the object
        :return: returns the id of the new panda robot
        """
        if position is None:
            position = [0.0, 0.0, 0.88]
        if orientation is None:
            orientation = [0.0, 0.0, 0.0]
        if path_to_urdf is None:
            obj_urdf = sim_framework_path(
                "./envs/frankaemika/robots/panda_arm_hand.urdf")  # panda robot with inhand camera
            # obj_urdf = sim_framework_path("./envs/frankaemika/robots/panda_arm_hand_pybullet.urdf")  # panda robot with inhand camera
            # obj_urdf  = sim_framework_path("./envs/frankaemika/robots/panda_arm_hand_without_cam.urdf")
            # obj_urdf  = sim_framework_path("./envs/frankaemika/robots/panda_arm_hand_without_cam_inertia_from_mujoco.urdf")
        else:
            obj_urdf = path_to_urdf
        orientation = list(orientation)
        if len(orientation) == 3:
            orientation = p.getQuaternionFromEuler(orientation)
        position = list(position)

        try:
            id = p.loadURDF(obj_urdf,
                            position,
                            orientation,
                            useFixedBase=1,
                            # | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
                            flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE,
                            # flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT,
                            physicsClientId=self.physics_client_id)

            ik_id = p.loadURDF(obj_urdf,
                               position,
                               orientation,
                               useFixedBase=1,
                               flags=p.URDF_USE_SELF_COLLISION,  # | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
                               physicsClientId=self.ik_client_id)

            if id_name is not None:
                setattr(self, id_name + '_id', id)
                self.obj_name2id[id_name + '_id'] = id
            else:
                self.robot_physics_client_id = id
                self.robot_ik_client_id = ik_id

        except Exception:
            print()
            print('Stopping the program')
            raise ValueError('Could not load URDF-file: Check the path to file. Stopping the program. Your path:',
                             obj_urdf)
        if init_q is None:
            init_q = (3.57795216e-09,
                      1.74532920e-01,
                      3.30500960e-08,
                      -8.72664630e-01,
                      -1.14096181e-07,
                      1.22173047e+00,
                      7.85398126e-01)

        self.set_q(init_q, id)

        # robotEndEffectorIndex = 8
        # robotEndEffectorIndex = 9
        robotEndEffectorIndex = 12
        return id, ik_id, robotEndEffectorIndex

    def load_object_to_scene(self, path_to_urdf, orientation, position, id_name, fixed=0, inertia_from_file=False):
        """
        This function loads an object to the simulation environment. If loading the object fails, the program is stopped
        and an appropriate get_error message is returned to user.

        :param path_to_urdf: the whole path of the location of the urdf file to be load as string
        :param orientation: orientation of the object. Can be either euler angles, or quaternions. NOTE: quaternions
                            notation: [x, y, z, w] !
        :param position: cartesian world position to place the object
        :param id_name: string valued name of the object. This name can then be called as self.'name'_id to get the
                        id number of the object
        :param fixed: if the object is fixed in the scene
        :param inertia_from_file: if the inertia values from file should be used, or pybullet should calculate its own
                                  inertia values.
        :return: returns the id of the loaded object
        """
        obj_urdf = path_to_urdf
        orientation = list(orientation)
        if len(orientation) == 3:
            orientation = p.getQuaternionFromEuler(orientation)
        position = list(position)

        try:
            if inertia_from_file == True:
                id = p.loadURDF(obj_urdf,
                                position,
                                orientation,
                                fixed,
                                flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE,
                                physicsClientId=self.physics_client_id)
            else:
                id = p.loadURDF(obj_urdf,
                                position,
                                orientation,
                                fixed,
                                flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.physics_client_id)

            setattr(self, id_name + '_id', id)
        except Exception:
            print()
            print('Stopping the program')
            raise ValueError('Could not load URDF-file: Check the path to file. Stopping the program. Your path:',
                             obj_urdf)
        return id

    def get_id_from_name(self, obj_name):
        """
        Returns the object id from the object name specified when creating the object.

        Args:
            obj_name: Name of the object

        Returns:
            Index of the object
        """
        return self.obj_name2id[obj_name + '_id']

    def load_graspa_layout_to_scene(self, path_to_sdf, orientation, position, id_name):
        """
        This function loads a GRASPA layout to the simulation environment. If loading the layout fails, the program is stopped
        and an appropriate get_error message is returned to user.
        :param path_to_sdf: the whole path of the location of the sdf file to be load as string
        :param orientation: orientation of the layout. Can be either euler angles, or quaternions. NOTE: quaternions
                            notation: [x, y, z, w] !
        :param position: cartesian world position of the layout. Ref frame is positioned on the bottom right corner
        :param id_name: string valued name of the layout. This name can then be called as self.'name'_id to get the
                        id number of the layout
        :return: returns the id of the loaded assets
        """

        layout_sdf = path_to_sdf
        try:
            objects_ids = p.loadSDF(layout_sdf)

        except Exception:
            print()
            print('Stopping the program')
            raise ValueError('Could not load SDF-file: Check the path to file. Stopping the program. Your path:',
                             layout_sdf)

        for i, id in enumerate(objects_ids):
            setattr(self, id_name + '_' + str(i) + '_id', id)

        orientation = list(orientation)
        if len(orientation) == 3:
            orientation = p.getQuaternionFromEuler(orientation)
        position = list(position)

        for obj in objects_ids:
            pose_obj = p.getBasePositionAndOrientation(obj)
            new_pose_obj = p.multiplyTransforms(position, orientation, pose_obj[0], pose_obj[1])
            p.resetBasePositionAndOrientation(obj, new_pose_obj[0], new_pose_obj[1])

        matrix = p.getMatrixFromQuaternion(orientation)
        dcm = np.array([matrix[0:3], matrix[3:6], matrix[6:9]])
        pax = np.add(position, dcm.dot([0.1, 0, 0]))
        pay = np.add(position, dcm.dot([0, 0.1, 0]))
        paz = np.add(position, dcm.dot([0, 0, 0.1]))

        p.addUserDebugLine(position, pax.tolist(), [1, 0, 0])
        p.addUserDebugLine(position, pay.tolist(), [0, 1, 0])
        p.addUserDebugLine(position, paz.tolist(), [0, 0, 1])

        return objects_ids

    def set_q(self, joints, robot_id=None, physicsClientId=None):
        """
        Sets the value of the robot joints.
        WARNING: This overrides the physics, do not use during simulation!!

        :param joints: tuple of size (7)
        :return: no return value
        """
        if physicsClientId is None:
            physicsClientId = self.physics_client_id
        if robot_id is None:
            robot_id = self.robot_physics_client_id
        j1, j2, j3, j4, j5, j6, j7 = joints

        joint_angles = {}
        joint_angles["panda_joint_world"] = 0.0  # No actuation
        joint_angles["panda_joint1"] = j1
        joint_angles["panda_joint2"] = j2
        joint_angles["panda_joint3"] = j3
        joint_angles["panda_joint4"] = j4
        joint_angles["panda_joint5"] = j5
        joint_angles["panda_joint6"] = j6
        joint_angles["panda_joint7"] = j7
        joint_angles["panda_joint8"] = 0.0  # No actuation
        joint_angles["panda_hand_joint"] = 0.0  # No actuation
        joint_angles["panda_finger_joint1"] = 0.05
        joint_angles["panda_finger_joint2"] = 0.05
        joint_angles["panda_grasptarget_hand"] = 0.0
        joint_angles["camera_joint"] = 0.0  # No actuation
        joint_angles["camera_depth_joint"] = 0.0  # No actuation
        joint_angles["camera_depth_optical_joint"] = 0.0  # No actuation
        joint_angles["camera_left_ir_joint"] = 0.0  # No actuation
        joint_angles["camera_left_ir_optical_joint"] = 0.0  # No actuation
        joint_angles["camera_right_ir_joint"] = 0.0  # No actuation
        joint_angles["camera_right_ir_optical_joint"] = 0.0  # No actuation
        joint_angles["camera_color_joint"] = 0.0  # No actuation
        joint_angles["camera_color_optical_joint"] = 0.0  # No actuation

        for joint_index in range(p.getNumJoints(robot_id, physicsClientId=physicsClientId)):
            joint_name = p.getJointInfo(robot_id, joint_index, physicsClientId=physicsClientId)[1].decode('ascii')
            joint_angle = joint_angles.get(joint_name, 0.0)
            # self.physics_client.changeDynamics(robot_id, joint_index, linearDamping=0, angularDamping=0)
            p.resetJointState(bodyUniqueId=robot_id,
                              jointIndex=joint_index,
                              targetValue=joint_angle,
                              physicsClientId=physicsClientId)

    def get_point_cloud_inHandCam(self, robot_id=None):
        """
        Calculates the 3d world coordinates and the corresponding rgb values of inHandCamera.

        :param plot_points: whether the points shall be plot in matplotlib (very slow)
        :param robot_id: if not set, it will be set to the robot id of the robot within the physics client
        :return: points: numpy array (nb_points x 3)
                 colors: numpy array (nb_points x 4)
        """
        if robot_id is None:
            robot_id = self.robot_physics_client_id
        client_id = self.physics_client_id
        points, colors = self.inhand_cam.calc_point_cloud(id=robot_id, client_id=client_id)
        return points, colors

    def get_point_cloud_CageCam(self, cam_id):
        """
        Calculates the 3d world coordinates and the corresponding rgb values.

        :param cam_id: the id of the cage camera.
        :param plot_points:whether the points shall be plot in matplotlib (very slow)
        :return: 3d world coordinates and the corresponding rgb values
        """
        client_id = self.physics_client_id
        points, colors = self.cage_cam.calc_point_cloud(id=cam_id, client_id=client_id)
        return points, colors

    def get_segmentation_from_cam(self, cam_id, client_id=None, with_noise=False, shadow=True):
        if client_id is None:
            client_id = self.physics_client_id
        try:
            _, _, seg_img = self.cage_cam.get_image(cam_id=cam_id,
                                                    client_id=client_id,
                                                    with_noise=with_noise,
                                                    shadow=shadow)
            return seg_img
        except ValueError:
            print("Error, no camera with id " + str(cam_id))

    def get_depth_image_from_cam(self, cam_id, client_id=None, with_noise=False, shadow=True):
        if client_id is None:
            client_id = self.physics_client_id
        try:
            _, depth_img, _ = self.cage_cam.get_image(cam_id=cam_id,
                                                      client_id=client_id,
                                                      with_noise=with_noise,
                                                      shadow=shadow)
            return depth_img
        except ValueError:
            print("Error, no camera with id " + str(cam_id))

    def get_rgb_image_from_cam(self, cam_id, client_id=None, with_noise=False, shadow=True):
        if client_id is None:
            client_id = self.physics_client_id
        try:
            rgb_img, _, _ = self.cage_cam.get_image(cam_id=cam_id,
                                                    client_id=client_id,
                                                    with_noise=with_noise,
                                                    shadow=shadow)
            return rgb_img
        except ValueError:
            print("Error, no camera with id " + str(cam_id))

    def get_point_cloud_from_cam(self, cam_id, client_id=None, with_noise=False):
        if client_id is None:
            client_id = self.physics_client_id
        try:
            points, colors = self.cage_cam.calc_point_cloud(id=cam_id,
                                                            client_id=client_id,
                                                            with_noise=with_noise)
            return points, colors
        except ValueError:
            print("Error, no camera with id " + str(cam_id))

    def disable_robot_vel_ctrl(self, robot_id):
        p.setJointMotorControlArray(robot_id,
                                    list(np.arange(1, 8)),
                                    p.VELOCITY_CONTROL,
                                    forces=list(np.zeros(7)),
                                    physicsClientId=self.physics_client_id)

    def enable_robot_vel_ctrl(self, robot_id, max_forces):
        p.setJointMotorControlArray(robot_id,
                                    list(np.arange(1, 8)),
                                    p.VELOCITY_CONTROL,
                                    forces=list(max_forces),
                                    physicsClientId=self.physics_client_id)
