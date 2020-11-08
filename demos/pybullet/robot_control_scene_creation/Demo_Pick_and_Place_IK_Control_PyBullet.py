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
import pybullet as p
from classic_framework.pybullet.PyBulletRobot import PyBulletRobot
from classic_framework.pybullet.PyBulletScene import PyBulletScene as Scene
from classic_framework.interface.Logger import RobotPlotFlags
from classic_framework.pybullet.pb_utils.pybullet_scene_object import PyBulletObject

if __name__ == '__main__':

    cube1 = PyBulletObject(urdf_name='cuboid',
                           object_name='cube_1',
                           position=[6.5e-01, 0.01, 0.91],
                           orientation=[np.pi / 2, 0, 0],
                           data_dir=None)

    cube2 = PyBulletObject(urdf_name='cuboid',
                           object_name='cube_2',
                           position=[6.5e-01, -0.2, 0.91],
                           orientation=[np.pi / 2, 0, 0],
                           data_dir=None)

    duck = PyBulletObject(urdf_name='duck_vhacd',
                          object_name='duck1',
                          position=[6.7e-01, 0.2, 0.91],
                          orientation=[np.pi / 2, 0, 0],
                          data_dir=None)
    # load duck

    object_list = [cube1, cube2, duck]
    scene = Scene(object_list=object_list)

    PyBulletRobot = PyBulletRobot(p, scene, gravity_comp=True)
    PyBulletRobot.use_inv_dyn = False

    init_pos = PyBulletRobot.current_c_pos
    init_or = PyBulletRobot.current_c_quat
    init_joint_pos = PyBulletRobot.current_j_pos

    PyBulletRobot.startLogging()
    # duration = 4
    duration = 2

    PyBulletRobot.ctrl_duration = duration
    PyBulletRobot.set_gripper_width = 0.04

    # desired_cart_pos_1 = np.array([6.5e-01, 0.01, 1])
    desired_cart_pos_1 = np.array([6.5e-01, 0.01, 0.91])
    # desired_quat_1 = [0.01806359,  0.91860348, -0.38889658, -0.06782891]
    desired_quat_1 = [0, 1, 0, 0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)

    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_1, desiredQuat=desired_quat_1, duration=duration)
    PyBulletRobot.set_gripper_width = 0.0
    print('now pos 1')

    desired_cart_pos_2 = [0.27909151, -0.09255804, 1.04587281]
    # desired_quat_2 = [0.01806359,  0.91860348, -0.38889658, -0.06782891]
    desired_quat_2 = [0, 1, 0, 0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_2, desiredQuat=desired_quat_2, duration=duration)
    PyBulletRobot.set_gripper_width = 0.04
    print('now pos 2')
    #
    # desired_cart_pos_3 = [6.6e-01, -0.2, 1]
    desired_cart_pos_3 = [6.6e-01, -0.2, 0.91]
    # desired_quat_3 = [0.01806359,  0.91860348, -0.38889658, -0.06782891]
    desired_quat_3 = [0, 1, 0, 0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_3, desiredQuat=desired_quat_3, duration=duration)
    PyBulletRobot.set_gripper_width = 0.0
    print('now pos 3')

    # desired_cart_pos_4 = [0.26, -0.09343867, 1.0957284]
    desired_cart_pos_4 = [0.26, -0.09343867, 1.0057284]
    # desired_quat_4 = [ 4.82629956e-04,  9.25339282e-01, -3.79132301e-01, -2.37905397e-03]
    desired_quat_4 = [0, 1, 0, 0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_4, desiredQuat=desired_quat_4, duration=duration)
    PyBulletRobot.set_gripper_width = 0.04
    print('now pos 4')

    # desired_cart_pos_5 = [0.71878368, 0.13568448, 1.08492804]
    desired_cart_pos_5 = [0.71878368, 0.13568448, 0.9949280400000001]
    # desired_quat_5 = [-0.05706405,  0.89564598, -0.43261695,  0.08604956]
    desired_quat_5 = [0, 1, 0, 0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_5, desiredQuat=desired_quat_5, duration=duration)
    print('now pos 5')

    # desired_cart_pos_6 = [0.64669174, 0.21333255, 1.02646971]
    desired_cart_pos_6 = [0.64669174, 0.21333255, 0.93646971]
    # desired_quat_6 = [-0.02874837,  0.92223102, -0.3835102 ,  0.03979226]
    desired_quat_6 = [0, 1, 0, 0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_6, desiredQuat=desired_quat_6, duration=duration)
    PyBulletRobot.set_gripper_width = 0.0
    print('now pos 6')

    # desired_cart_pos_7 = [0.24392071, -0.10585493, 1.14383709]
    desired_cart_pos_7 = [0.24392071, -0.10585493, 1.0538370899999998]
    # desired_quat_7 = [-0.02043033,  0.90203577, -0.43093273,  0.01452782]
    desired_quat_7 = [0, 1, 0, 0]  # we use w,x,y,z. where pybullet uses x,y,z,w (we just have to swap the positions)
    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=desired_cart_pos_7, desiredQuat=desired_quat_7, duration=duration)
    PyBulletRobot.set_gripper_width = 0.04
    print('now pos 7')

    PyBulletRobot.gotoCartPositionAndQuat(desiredPos=init_pos, desiredQuat=init_or, duration=duration)
    print('now pos 8')

    PyBulletRobot.stopLogging()
    PyBulletRobot.logger.plot(RobotPlotFlags.END_EFFECTOR | RobotPlotFlags.JOINTS)
