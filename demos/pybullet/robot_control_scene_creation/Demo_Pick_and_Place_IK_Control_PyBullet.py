"""
Demo for picking and placing assets (2 x cube, 1 x duck) using inverse kinematics.

"Call Stack" for the Pick and Place inverse kinematics control demos:
1. Load Scene and Robot (init of the robot invokes receiveState())
2. PyBulletRobot.gotoCartPositionAndQuat (give desired cartesian coords and quaternion for orientation)
        2.1 get_qdq_J, get_x
        2.2 use fictive robot to obtain desired joint positions (des_joints)
        2.3 PyBulletRobot.gotoJointPositions (give desired joint positions)
            2.3.1 setDesiredPositions
            2.3.2 executeController
                2.3.2.1 robot.nextStep
                2.3.2.2 robot.receiveState
                2.3.2.3 while not finished: getControl and nextStep
"""
import numpy as np

from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.pybullet.pb_utils.pybullet_scene_object import PyBulletURDFObject

if __name__ == "__main__":
    ### Create Objects
    cube1 = PyBulletURDFObject(
        urdf_name="cuboid",
        object_name="cube_1",
        position=[6.5e-01, 0.01, 0.91],
        orientation=[np.pi / 2, 0, 0],
        data_dir=None,
    )

    cube2 = PyBulletURDFObject(
        urdf_name="cuboid",
        object_name="cube_2",
        position=[6.5e-01, -0.2, 0.91],
        orientation=[np.pi / 2, 0, 0],
        data_dir=None,
    )

    duck = PyBulletURDFObject(
        urdf_name="duck_vhacd",
        object_name="duck1",
        position=[6.7e-01, 0.2, 0.91],
        orientation=[np.pi / 2, 0, 0],
        data_dir=None,
    )
    object_list = [cube1, cube2, duck]

    ### Create Robot and Scene. REQUIRED!
    sim_factory = SimRepository.get_factory("pybullet")

    scene = sim_factory.create_scene(object_list=object_list)
    pb_robot = sim_factory.create_robot(scene)
    scene.start()

    init_pos = pb_robot.current_c_pos
    init_or = pb_robot.current_c_quat
    init_joint_pos = pb_robot.current_j_pos

    scene.start_logging()
    duration = 4

    pb_robot.open_fingers()

    # desired_cart_pos_1 = np.array([6.5e-01, 0.01, 1])
    desired_cart_pos_1 = np.array([6.5e-01, 0.01, 0.015])
    # desired_quat_1 = [0.01806359,  0.91860348, -0.38889658, -0.06782891]
    desired_quat_1 = [
        0,
        1,
        0,
        0,
    ]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)

    pb_robot.gotoCartPositionAndQuat(
        desiredPos=desired_cart_pos_1, desiredQuat=desired_quat_1, duration=duration
    )
    pb_robot.close_fingers()
    print("now pos 1")

    desired_cart_pos_2 = [0.27909151, -0.09255804, 1.04587281]
    # desired_quat_2 = [0.01806359,  0.91860348, -0.38889658, -0.06782891]
    desired_quat_2 = [
        0,
        1,
        0,
        0,
    ]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    pb_robot.gotoCartPositionAndQuat(
        desiredPos=desired_cart_pos_2, desiredQuat=desired_quat_2, duration=duration
    )
    pb_robot.open_fingers()
    print("now pos 2")
    #
    # desired_cart_pos_3 = [6.6e-01, -0.2, 1]
    desired_cart_pos_3 = [6.6e-01, -0.2, 0.015]
    # desired_quat_3 = [0.01806359,  0.91860348, -0.38889658, -0.06782891]
    desired_quat_3 = [
        0,
        1,
        0,
        0,
    ]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    pb_robot.gotoCartPositionAndQuat(
        desiredPos=desired_cart_pos_3, desiredQuat=desired_quat_3, duration=duration
    )
    pb_robot.close_fingers()
    print("now pos 3")

    # desired_cart_pos_4 = [0.26, -0.09343867, 1.0957284]
    desired_cart_pos_4 = [0.26, -0.09343867, 1.0057284]
    # desired_quat_4 = [ 4.82629956e-04,  9.25339282e-01, -3.79132301e-01, -2.37905397e-03]
    desired_quat_4 = [
        0,
        1,
        0,
        0,
    ]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    pb_robot.gotoCartPositionAndQuat(
        desiredPos=desired_cart_pos_4, desiredQuat=desired_quat_4, duration=duration
    )
    pb_robot.open_fingers()
    print("now pos 4")

    # desired_cart_pos_5 = [0.71878368, 0.13568448, 1.08492804]
    desired_cart_pos_5 = [0.67, 0.2, 0.015]
    # desired_quat_5 = [-0.05706405,  0.89564598, -0.43261695,  0.08604956]
    desired_quat_5 = [
        0,
        1,
        0,
        0,
    ]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    pb_robot.gotoCartPositionAndQuat(
        desiredPos=desired_cart_pos_5, desiredQuat=desired_quat_5, duration=duration
    )
    print("now pos 5")

    # desired_cart_pos_6 = [0.64669174, 0.21333255, 1.02646971]
    desired_cart_pos_6 = [0.26, -0.09343867, 1.0057284]
    # desired_quat_6 = [-0.02874837,  0.92223102, -0.3835102 ,  0.03979226]
    desired_quat_6 = [
        0,
        1,
        0,
        0,
    ]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    pb_robot.gotoCartPositionAndQuat(
        desiredPos=desired_cart_pos_6, desiredQuat=desired_quat_6, duration=duration
    )
    pb_robot.close_fingers()
    print("now pos 6")

    pb_robot.gotoCartPositionAndQuat(
        desiredPos=init_pos, desiredQuat=init_or, duration=duration
    )
    print("now pos 8")

    scene.stop_logging()
    pb_robot.robot_logger.plot(RobotPlotFlags.END_EFFECTOR | RobotPlotFlags.JOINTS)
