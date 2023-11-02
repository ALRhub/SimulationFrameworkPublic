from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box

if __name__ == "__main__":
    free_box = Box(
        name="free_box",
        init_pos=[0.6, -0.2, 0.15],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 0, 0, 1],
    )

    static_box = Box(
        name="static_box",
        init_pos=[0.6, -0.2, 0.15],
        init_quat=[0, 1, 0, 0],
        rgba=[0, 1, 0, 1],
        static=True,
        visual_only=True,
    )

    object_list = [free_box, static_box]
    duration = 4

    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)

    scene.start()

    home_position = robot.current_c_pos
    home_quat = robot.current_c_quat

    scene.start_logging()  # log the data for plotting

    robot.set_desired_gripper_width(0.04)
    robot.gotoCartPositionAndQuat_ImpedanceCtrl(
        desiredPos=[0.60, -0.20, 0.01],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=3.0,
    )

    robot.wait(duration=1.0)

    orig_box_pos = scene.get_obj_pos(free_box)
    orig_box_quat = scene.get_obj_quat(free_box)

    """Close the fingers."""

    robot.close_fingers(duration=0.5)

    """Go back to inital position, let the fingers closed."""

    robot.gotoCartPositionAndQuat_ImpedanceCtrl(
        desiredPos=home_position, desiredQuat=home_quat, duration=3.0
    )
    robot.wait(duration=1.0)

    robot.set_desired_gripper_width(0.04)

    robot.wait(duration=2.0)

    ########## Moving the green box to the landing spot of the red box, then moving the red box back to its starting pose ##############
    landed_pos = scene.get_obj_pos(free_box)
    landed_quat = scene.get_obj_quat(free_box)

    scene.set_obj_pos_and_quat(
        new_pos=landed_pos, new_quat=landed_quat, sim_obj=static_box
    )
    scene.set_obj_pos_and_quat(
        new_pos=orig_box_pos, new_quat=orig_box_quat, sim_obj=free_box
    )
    ##################################

    robot.wait(duration=4)

    scene.stop_logging()
    robot.robot_logger.plot(
        plot_selection=RobotPlotFlags.END_EFFECTOR | RobotPlotFlags.JOINTS
    )
