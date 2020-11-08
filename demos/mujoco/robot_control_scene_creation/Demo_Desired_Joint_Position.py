import numpy as np

from classic_framework.interface.Logger import RobotPlotFlags
from classic_framework.mujoco.MujocoRobot import MujocoRobot
from classic_framework.mujoco.MujocoScene import MujocoScene as Scene
from classic_framework.mujoco.mujoco_utils.mujoco_scene_object import MujocoPrimitiveObject

if __name__ == '__main__':
    box = MujocoPrimitiveObject(obj_pos=[.6, -0.2, 0.15], obj_name="box", diaginertia=[0.01, 0.01, 0.01],
                                geom_rgba=[1, 0, 0, 1])
    object_list = [box]
    duration = 4
    # Setup the scene
    scene = Scene(object_list=object_list)

    mj_Robot = MujocoRobot(scene, gravity_comp=True, num_DoF=7)
    mj_Robot.ctrl_duration = duration

    robot_init_q = mj_Robot.current_j_pos
    mj_Robot.startLogging()  # log the data for plotting

    des_q = np.array([-0.17372284, 0.74377287, -0.15055875, -1.8271288, 0.17003154, 2.52458572, 1.85687575])
    mj_Robot.set_gripper_width = 0.04
    mj_Robot.gotoJointPosition(des_q, duration=duration)

    """Close the fingers."""
    duration = 3
    mj_Robot.ctrl_duration = duration
    mj_Robot.set_gripper_width = 0.0
    mj_Robot.gotoJointPosition(des_q, duration=3)

    """Go back to inital position, let the fingers closed."""
    duration = 4
    mj_Robot.ctrl_duration = duration
    mj_Robot.gotoJointPosition(robot_init_q, duration=duration)

    mj_Robot.stopLogging()
    mj_Robot.logger.plot(plotSelection=RobotPlotFlags.JOINTS)
