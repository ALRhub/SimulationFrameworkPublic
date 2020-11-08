"""
Demo for picking and placing assets (2 x cube, 1 x duck) using joint control.

"Call Stack" for the Pick and Place joint control demos:
1. Load Scene and Robot (init of the robot invokes receiveState())
2. PyBulletRobot.gotoJointPositions (give desired joint positions)
        2.1 setDesiredPositions
        2.2 executeController
            2.2.1 robot.nextStep
            2.2.2 robot.receiveState
            2.2.3 while not finished: getControl and nextStep
"""
import numpy as np
import pybullet as p

from classic_framework.pybullet.PyBulletRobot import PyBulletRobot
from classic_framework.pybullet.PyBulletScene import PyBulletScene as Scene
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

    object_list = [cube1, cube2, duck]
    scene = Scene(object_list=object_list)
    PyBulletRobot = PyBulletRobot(p, scene, gravity_comp=True)

    init_joint_pos = PyBulletRobot.current_j_pos

    PyBulletRobot.startLogging()
    duration = 4

    PyBulletRobot.ctrl_duration = duration

    PyBulletRobot.set_gripper_width = 0.04

    target_j_1 = np.array([-1.985311177161578e-08,
                           0.73,
                           1.8712587204686806e-08,
                           -1.8413169784398997,
                           -4.465672043033017e-08,
                           2.6012887413098415,
                           0.7853981960087343])

    PyBulletRobot.gotoJointPosition(target_j_1, duration=duration)
    print('position 1 get_error:', PyBulletRobot.get_joint_pos_error(target_j_1))

    PyBulletRobot.set_gripper_width = 0.0

    target_j_2 = np.array([-0.16567619083577795,
                           -0.0057623624514093,
                           -0.17176196926016699,
                           -2.9973142009581024,
                           -0.1012804365932158,
                           2.818177135153361,
                           0.5482779879510586])

    PyBulletRobot.gotoJointPosition(target_j_2, duration=duration)
    print('position 2 get_error:', PyBulletRobot.get_joint_pos_error(target_j_2))
    PyBulletRobot.set_gripper_width = 0.04

    target_j_3 = np.array([-0.26449829864544533,
                           0.86,
                           -0.041667572046128046,
                           -1.500104903732597,
                           0.06158936782037842,
                           2.25,
                           0.4563119728880853])

    PyBulletRobot.gotoJointPosition(target_j_3, duration=duration)
    print('position 3 get_error:', PyBulletRobot.get_joint_pos_error(target_j_3))
    PyBulletRobot.set_gripper_width = 0.0

    target_j_4 = np.array([-0.16567619083577795,
                           -0.20057623624514093,
                           -0.17176196926016699,
                           -2.9973142009581024,
                           -0.1012804365932158,
                           2.818177135153361,
                           0.5482779879510586])

    PyBulletRobot.gotoJointPosition(target_j_4, duration=duration)
    print('position 4 get_error:', PyBulletRobot.get_joint_pos_error(target_j_4))
    PyBulletRobot.set_gripper_width = 0.04

    target_j_5 = np.array([0.15,
                           0.8,
                           0.04150050261642385,
                           -1.5,
                           -0.06676227707945848,
                           2.5575496640396715,
                           1.142662332243999])

    PyBulletRobot.gotoJointPosition(target_j_5, duration=duration)
    print('position 5 get_error:', PyBulletRobot.get_joint_pos_error(target_j_5))
    PyBulletRobot.set_gripper_width = 0.04

    target_j_6 = np.array([0.2833209698967932,
                           0.8,
                           0.04150050261642385,
                           -1.7,
                           -0.06676227707945848,
                           2.5575496640396715,
                           1.142662332243999])

    PyBulletRobot.gotoJointPosition(target_j_6, duration=duration)
    print('position 6 get_error:', PyBulletRobot.get_joint_pos_error(target_j_6))
    PyBulletRobot.set_gripper_width = 0.0

    target_j_7 = np.array([-0.3007619083577795,
                           -0.6,
                           -0.17176196926016699,
                           -3.2,
                           -0.1012804365932158,
                           2.6,
                           0.5482779879510586])

    PyBulletRobot.gotoJointPosition(target_j_7, duration=duration)
    print('position 7 get_error:', PyBulletRobot.get_joint_pos_error(target_j_7))
    PyBulletRobot.set_gripper_width = 0.04
    PyBulletRobot.gotoJointPosition(init_joint_pos, duration=duration)
    print('position 8 get_error:', PyBulletRobot.get_joint_pos_error(init_joint_pos))


    PyBulletRobot.stopLogging()
    PyBulletRobot.logger.stopLogging()
    PyBulletRobot.logger.plot()
