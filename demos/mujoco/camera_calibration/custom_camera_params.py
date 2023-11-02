import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import transforms3d as t3d

from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box
from alr_sim.utils.geometric_transformation import quat2mat


def show_image(image):
    plt.imshow(image)
    plt.axis("off")
    plt.show()


def posRotMat2Mat(pos, rot_mat):
    t_mat = np.eye(4)
    t_mat[:3, :3] = rot_mat
    t_mat[:3, 3] = np.array(pos)
    return t_mat


if __name__ == "__main__":
    box = Box(
        name="box",
        init_pos=[0.6, -0.2, 0.15],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 0, 0, 1],
    )

    box2 = Box(
        name="box2",
        init_pos=[0.6, 0.2, 0.15],
        init_quat=[0, 1, 0, 0],
        rgba=[0, 1, 0, 1],
    )

    object_list = [box, box2]
    duration = 4

    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)

    robot.inhand_cam.set_cam_params(height=480, width=640)

    scene.start()
    scene.start_logging()

    robot.gotoJointPosition(
        np.array(
            [
                4.25747654e-04,
                -9.12916288e-02,
                1.65991567e-03,
                -1.32146442e00,
                -6.92314841e-03,
                1.21604252e00,
                7.82988906e-01,
            ]
        )
    )

    home_position = robot.current_c_pos
    home_quat = robot.current_c_quat

    robot.activeController = robot.cartesianPosQuatTrackingController

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6, origin=[0, 0, 0]
    )

    inhand_cam = robot.inhand_cam

    # Compute world to camera transformation matrix
    cam_pos, cam_quat = inhand_cam.get_cart_pos_quat()
    cam_tf = posRotMat2Mat(cam_pos, quat2mat(cam_quat))

    rgb_image, depth_image = inhand_cam.get_image()
    points, colors = inhand_cam.calc_point_cloud()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # transformed_cloud = pcd.transform(cam_tf)

    o3d.visualization.draw_geometries([pcd, mesh_frame])

    show_image(rgb_image)
    show_image(depth_image)

    """Close the fingers."""
    robot.wait(duration=40.0)
    robot.close_fingers(duration=0.5)

    """Go back to inital position, let the fingers closed."""

    robot.gotoCartPositionAndQuat_ImpedanceCtrl(
        desiredPos=home_position, desiredQuat=home_quat, duration=4.0
    )
    robot.wait(duration=4.0)

    scene.stop_logging()
    robot.robot_logger.plot(
        plot_selection=RobotPlotFlags.END_EFFECTOR | RobotPlotFlags.JOINTS
    )
