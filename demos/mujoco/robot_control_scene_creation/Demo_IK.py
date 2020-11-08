import classic_framework.mujoco.mujoco_utils.mujoco_controllers as mj_ctrl
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
    # ctrl = mj_ctrl.IKControl()# torque control
    ctrl = mj_ctrl.MocapControl()  # mocap control
    scene = Scene(object_list=object_list, control=ctrl)  # mocap control

    mj_Robot = MujocoRobot(scene, gravity_comp=True, num_DoF=7)
    mj_Robot.ctrl_duration = duration

    home_position = mj_Robot.current_c_pos
    mj_Robot.startLogging()  # log the data for plotting

    mj_Robot.set_gripper_width = 0.04
    mj_Robot.gotoCartPositionAndQuat(desiredPos=[0.6, -0.20, 0.02], desiredQuat=[0.009, 0.72, -0.67, -0.014],
                                     duration=duration)
    """Close the fingers."""
    duration = 1
    mj_Robot.ctrl_duration = duration
    mj_Robot.set_gripper_width = 0.0
    mj_Robot.gotoCartPositionAndQuat(desiredPos=[0.6, -0.20, 0.02], desiredQuat=[0.009, 0.72, -0.67, -0.014],
                                     duration=duration)

    """Go back to inital position, let the fingers closed."""
    duration = 4
    mj_Robot.ctrl_duration = duration
    mj_Robot.gotoCartPositionAndQuat(desiredPos=home_position, desiredQuat=[0, 1, 0, 0], duration=duration)

    mj_Robot.stopLogging()
    mj_Robot.logger.plot(plotSelection=RobotPlotFlags.END_EFFECTOR)
