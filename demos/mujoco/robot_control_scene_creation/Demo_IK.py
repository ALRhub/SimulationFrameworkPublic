from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box

if __name__ == "__main__":
    box = Box(
        name="box",
        init_pos=[0.6, -0.2, 0.15],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 0, 0, 1],
    )
    object_list = [box]
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
        duration=4.0,
    )
    """Close the fingers."""
    robot.wait(duration=4.0)
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
