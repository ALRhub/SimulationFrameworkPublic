import numpy as np
import pybullet as p

from classic_framework import RobotPlotFlags
from classic_framework.pybullet.PyBulletRobot import PyBulletRobot
from classic_framework.pybullet.PyBulletScene import PyBulletScene as Scene
from classic_framework.utils.sim_path import sim_framework_path

if __name__ == '__main__':
    data_dir = sim_framework_path('./envs/data/')

    scene = Scene(object_list=None)

    PyBulletRobot = PyBulletRobot(p, scene, gravity_comp=True)
    PyBulletRobot.use_inv_dyn = False

    init_pos = PyBulletRobot.current_c_pos
    init_or = PyBulletRobot.current_c_quat
    init_joint_pos = PyBulletRobot.current_j_pos

    PyBulletRobot.startLogging()
    duration = 8
    # duration = 2

    PyBulletRobot.ctrl_duration = duration
    PyBulletRobot.set_gripper_width = 0.04

    desired_cart_pos = np.array([6.32666683e-01, 1.63784830e-09, 1.2])
    desired_quat = np.array([0.7071067811865476, 0.7071067811865475, 0.0, 0.0])  # [w, ,x , y, z]
    # PyBulletRobot.set_desired_fing_pos(desiredPos=0)
    PyBulletRobot.set_gripper_width = 0.0
    PyBulletRobot.gotoCartPositionAndQuat(desired_cart_pos, desired_quat, use_fictive_robot=False, duration=duration)
    PyBulletRobot.stopLogging()
    PyBulletRobot.logger.plot(RobotPlotFlags.END_EFFECTOR | RobotPlotFlags.JOINTS | RobotPlotFlags.CTRL_TORQUES |
                              RobotPlotFlags.GRAVITY)
