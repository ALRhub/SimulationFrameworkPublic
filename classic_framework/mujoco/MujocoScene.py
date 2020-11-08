import os

import numpy as np
from mujoco_py import MjSim, MjViewer, load_model_from_path, MjSimState

from classic_framework.interface.Scene import Scene
from classic_framework.mujoco.Mujoco_Camera import Camera

import classic_framework.mujoco.mujoco_utils.mujoco_helpers as mj_help
from classic_framework.mujoco.mujoco_utils.mujoco_scene_object import \
    MujocoLoadable
from classic_framework.mujoco.mujoco_utils.mujoco_scene_parser import \
    MujocoSceneParser
import classic_framework.mujoco.mujoco_utils.mujoco_controllers as mj_ctrl
from classic_framework.utils.sim_path import sim_framework_path


class MujocoScene(Scene):
    def __init__(self,
                 object_list=None,
                 camera_list=None,
                 panda_xml_path=None,
                 control: mj_ctrl.MujocoController = None,
                 gripper_control=None,
                 dt=0.001,
                 n_substeps=1,
                 kp=None,
                 kp_scale=1,
                 kv=None,
                 kv_scale=1,
                 render=True):
        super(MujocoScene, self).__init__(object_list=object_list, dt=dt,
                                          render=render)
        """
        Initialises the scene.

        Args:
            object_list:
                List with :class:`MujocoObject`s
            camera_list:
                List with :class:`Camera`s
            control:
                A MujocoController implementation
            dt:
                1 / number of timesteps needed for computing one second of wall-clock time
            kv:
                Velocity feedback gain for each joint (list with num joints entries)
            kv_scale:
                Scales each Velocity feedback gain by a scalar (single value)
            kp:
                Position feedback gain for each joint (list with num joints entries)
            kp_scale:
                Scales each Position feedback gain by a scalar (single value)
        """
        if control is None:
            control = mj_ctrl.IKControl()
        elif isinstance(control, str):
            control = mj_ctrl.convert_str2control(control)

        if panda_xml_path is None:
            panda_xml_path = sim_framework_path('./envs/mujoco/panda/panda_with_cam_mujoco.xml')
            if control.ctrl_name == 'mocap':
                panda_xml_path = sim_framework_path('./envs/mujoco/panda/panda_mocap_model.xml')

        if camera_list is None:
            camera_list = []
        if object_list is None:
            object_list = []
        if not isinstance(object_list, list):
            object_list = [object_list]
        if not isinstance(camera_list, list):
            camera_list = [camera_list]

        self.control = control

        self.panda_xml_path = panda_xml_path

        self.xml_parser = MujocoSceneParser(panda_xml_path=panda_xml_path)
        self.n_substeps = n_substeps

        self.xml_parser.set_dt(self.dt)

        if control is not None:
            self.xml_parser.set_control(control=control,
                                        gripper_ctrl=gripper_control,
                                        kp=kp,
                                        kp_scale=kp_scale,
                                        kv=kv,
                                        kv_scale=kv_scale)

        self.init_qpos = None
        self.init_qvel = None
        self.body_to_idx = None
        self.joint_to_idx = None
        self.site_to_idx = None
        self.geom_to_idx = None

        self.camera_list = camera_list
        self.object_list = object_list
        self.sim, self.model, self.viewer = self.setup_scene()
        self.inhand_cam = Camera(self.sim, name='rgbd', fovy=45, ipd=0.068)
        self.cage_cam = Camera(self.sim, name='rgbd_cage', fovy=45, ipd=0.068)

        self.cameras = {'rgbd': self.inhand_cam,
                        'rgbd_cage': self.cage_cam}

        self.joint_names = [name for name in self.sim.model.joint_names if
                            name.startswith('panda_joint')]
        assert len(
            self.joint_names) == 7, "Error, found more joints than expected."

        self.gripper_names = [name for name in self.sim.model.joint_names if
                              name.startswith('panda_finger_joint')]
        assert len(
            self.gripper_names) == 2, "Error, found more gripper joints than expected."

        for cam in camera_list:
            self.cameras[cam.cam_name] = Camera(sim=self.sim, name=cam.cam_name,
                                                fovy=float(cam.fovy),
                                                ipd=cam.ipd)

    def setup_scene(self):
        """
        Creates a scene which is read from a .xml file.

        Returns:
            No return value
        """

        xml_path = sim_framework_path('./envs/mujoco/panda/main_scene.xml')

        # xml_path = sim_framework_path('./envs/mujoco/panda/panda_with_cam_original_inertia.xml')
        # ik control with orientation
        # has problems when using PD control,
        # when using inv dynamics control, works very good

        # adding all assets to the scene
        for obj in self.object_list:
            if isinstance(obj, MujocoLoadable):
                self.xml_parser.load_mj_loadable(obj)

        for cam in self.camera_list:
            self.xml_parser.load_mj_loadable(cam)

        model = load_model_from_path(xml_path)
        self.sim = MjSim(model=model, nsubsteps=self.n_substeps)

        self.geom_to_idx = mj_help.get_geom_to_idx_dict(model=model)
        self.body_to_idx = mj_help.get_body_to_idx_dict(model=model)
        self.joint_to_idx = mj_help.get_joint_to_idx_dict(model=model)
        self.site_to_idx = mj_help.get_site_to_idx_dict(model=model)

        self.load_panda_to_scene()

        os.remove(xml_path)  # remove the created xml file after creating the scene

        # Show simulation
        self.viewer = None
        if self.render:
            self.viewer = MjViewer(self.sim)
            self.viewer.render()
        return self.sim, model, self.viewer

    def load_panda_to_scene(self, robot_init_q=None):
        """
        Sets the initial joint position of the panda robot.

        Args:
            robot_init_q: numpy array (num dof,); initial joint positions

        Returns:
            No return value
        """
        self.init_qpos = self.sim.data.qpos.copy()

        if robot_init_q is None:
            robot_init_q = np.array([3.57795216e-09,
                                     1.74532920e-01,
                                     3.30500960e-08,
                                     -8.72664630e-01,
                                     -1.14096181e-07,
                                     1.22173047e+00,
                                     7.85398126e-01])
        self.set_q(robot_init_q)
        self.init_qpos[:robot_init_q.shape[0]] = robot_init_q
        self.init_qvel = np.zeros(self.sim.data.qvel.shape)

    def set_q(self, joint_pos):
        """
        Sets the value of the robot joints.
        Args:
            joint_pos: Value for the robot joints.

        Returns:
            No return value
        """
        qpos = self.sim.data.qpos.copy()
        qpos[:joint_pos.shape[0]] = joint_pos
        self.qpos = qpos
        self.qvel = self.sim.data.qvel.copy()
        self.qvel[:joint_pos.shape[0]] = np.zeros(joint_pos.shape[0])
        # self.qvel = np.zeros(self.sim.data.qvel.shape)
        mjSimState = MjSimState(time=0.0, qpos=self.qpos, qvel=self.qvel,
                                act=None, udd_state={})
        self.sim.set_state(mjSimState)

    def load_object_to_scene(self, obj):
        if isinstance(obj, MujocoLoadable):
            self.xml_parser.load_mj_loadable(obj)
        else:
            raise ValueError("Error, object has the implement <MujocoLoadable>")

    def get_id_from_name(self, obj_name):
        """
        Returns the object id from the object name specified when creating the object.

        Args:
            obj_name: Name of the object

        Returns:
            Index of the object
        """
        if obj_name in self.body_to_idx.keys():
            return self.body_to_idx(obj_name)
        elif obj_name in self.geom_to_idx.keys():
            return self.geom_to_idx(obj_name)
        elif obj_name in self.site_to_idx.keys():
            return self.site_to_idx(obj_name)
        elif obj_name in self.joint_to_idx.keys():
            return self.joint_to_idx(obj_name)

    def get_point_cloud_inHandCam(self, width=None, height=None):
        """
        Calculates the 3d world coordinates and the corresponding rgb values of inHandCamera.
        :return: points: numpy array (nb_points x 3)
                 colors: numpy array (nb_points x 4)
        """
        cam_name = 'rgbd'
        points, colors = self.cameras[cam_name].calc_point_cloud(width, height)
        return points, colors

    def get_point_cloud_CageCam(self, cam_name=None, width=None, height=None):
        """
        Calculates the 3d world coordinates and the corresponding rgb values of the cage camera.
        If cam_name is None, then use default camera which is already in the xml file.
        If cam_name is not None, make sure that its name is added as a camera in the xml file.
        :return: points: numpy array (nb_points x 3)
                 colors: numpy array (nb_points x 4)
        """
        if cam_name is None:
            cam_name = 'rgbd_cage'
        points, colors = self.cameras[cam_name].calc_point_cloud(width, height)
        return points, colors

    def get_segmentation_from_cam(self, cam_name, width=None, height=None):
        assert self.cameras.get(cam_name,
                                None), "Error, there exists no camera with the specified name."
        return self.cameras[cam_name].get_segmentation(width=width,
                                                       height=height)

    def get_depth_image_from_cam(self, cam_name, width=None, height=None):
        assert self.cameras.get(cam_name,
                                None), "Error, there exists no camera with the specified name."
        return self.cameras[cam_name].get_image(width=width, height=height)[1]

    def get_rgb_image_from_cam(self, cam_name, width=None, height=None):
        assert self.cameras.get(cam_name,
                                None), "Error, there exists no camera with the specified name."
        return self.cameras[cam_name].get_image(width=width, height=height)[0]

    def get_point_cloud_from_cam(self, cam_name):
        assert self.cameras.get(cam_name,
                                None), "Error, there exists no camera with the specified name."
        return self.cameras[cam_name].calc_point_cloud()

