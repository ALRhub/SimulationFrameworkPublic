from classic_framework.mujoco.MujocoRobot import MujocoRobot
from classic_framework.mujoco.MujocoScene import MujocoScene as Scene
from classic_framework.interface.Logger import RobotPlotFlags
from classic_framework.utils.sim_path import sim_framework_path

import numpy as np

if __name__ == '__main__':

    object_list = []
    duration = 4
    # Setup the scene
    scene = Scene(object_list=object_list)

    mj_Robot = MujocoRobot(scene, gravity_comp=True, num_DoF=7)
    mj_Robot.startLogging()
    # load the trajectory you want to follow
    path2file = sim_framework_path('demos/mujoco/robot_control_scene_creation/des_joint_traj.npy')
    des_joint_trajectory = np.load(path2file)
    mj_Robot.follow_JointTraj(des_joint_trajectory)

    mj_Robot.stopLogging()
    mj_Robot.logger.plot(plotSelection=RobotPlotFlags.JOINTS)