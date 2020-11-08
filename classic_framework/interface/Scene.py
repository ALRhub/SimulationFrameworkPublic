import open3d


class Scene:

    def __init__(self, object_list=None, dt=0.001, render=True):
        self.object_list = object_list
        self.dt = dt
        self.render = render
        self.point_cloud_visualizer = None
        self.point_cloud_visualizer_helper = None

    def setup_scene(self):
        raise NotImplementedError

    def load_panda_to_scene(self, init_q=None, orientation=None, position=None,
                            path_to_model=None):
        raise NotImplementedError

    def set_q(self, joints, robot_id=None, physicsClientId=None):
        raise NotImplementedError

    def get_id_from_name(self, obj_name):
        raise NotImplementedError

    def get_point_cloud_inHandCam(self, robot_id=None, width=None, height=None):
        raise NotImplementedError

    def get_point_cloud_CageCam(self, cam_id=None, cam_name=None, width=None,
                                height=None):
        raise NotImplementedError

    def get_segmentation_from_cam(self, cam_id=None, cam_name=None,
                                  client_id=None, with_noise=False, shadow=True,
                                  width=None, height=None):
        raise NotImplementedError

    def get_depth_image_from_cam(self, cam_id=None, cam_name=None,
                                 client_id=None, with_noise=False, shadow=True,
                                 width=None, height=None):
        raise NotImplementedError

    def get_rgb_image_from_cam(self, cam_id=None, cam_name=None, client_id=None,
                               with_noise=False, shadow=True,
                               width=None, height=None):
        raise NotImplementedError

    def visualize_point_clouds(self, points, colors):
        colors = colors[:, :3]
        if self.point_cloud_visualizer is None:
            self.point_cloud_visualizer = open3d.visualization.Visualizer()
            self.point_cloud_visualizer.create_window()
            self.point_cloud_visualizer_helper = open3d.geometry.PointCloud()
            self.point_cloud_visualizer_helper.points = open3d.utility.Vector3dVector(
                points)
            self.point_cloud_visualizer_helper.colors = open3d.utility.Vector3dVector(
                colors)
            self.point_cloud_visualizer.add_geometry(
                self.point_cloud_visualizer_helper)
        # open3d.visualization.draw_geometries([self.visualizer])
        else:
            # self.test.remove_geometry(self.visualizer)
            self.point_cloud_visualizer_helper.points = open3d.utility.Vector3dVector(
                points)
            self.point_cloud_visualizer_helper.colors = open3d.utility.Vector3dVector(
                colors)
            # open3d.visualization.draw_geometries([self.visualizer])
            self.point_cloud_visualizer.add_geometry(
                self.point_cloud_visualizer_helper)

        mesh_frame = open3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0, origin=[0, 0, 0])
        self.point_cloud_visualizer.add_geometry(mesh_frame)
        self.point_cloud_visualizer.run()
        self.point_cloud_visualizer.destroy_window()
        self.point_cloud_visualizer = None
        self.point_cloud_visualizer_helper = None
