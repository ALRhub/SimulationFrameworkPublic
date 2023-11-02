from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box
from alr_sim.controllers.Controller import (
    ModelBasedFeedforwardController,
)

if __name__ == "__main__":
    REAL_ROBOT = True

    duration = 4

    # Setup the scene
    sim_factory = SimRepository.get_factory("sl")

    s = sim_factory.create_scene()
    if REAL_ROBOT:
        robot = sim_factory.create_robot(
            s,
            robot_name="panda1",
            backend_addr="tcp://141.3.53.151:51468",
            local_addr="141.3.53.158",
            gripper_actuation=True,
        )
    else:
        robot = sim_factory.create_robot(s, robot_name="franka_m")

    s.start()

    home_position = robot.current_c_pos
    home_quat = robot.current_c_quat

    robot.jointTrackingController = ModelBasedFeedforwardController()
    s.start_logging()  # log the data for plotting

    robot.set_desired_gripper_width(0.04)
    robot.gotoCartPositionAndQuat_ImpedanceCtrl(
        desiredPos=[0.6, -0.20, 0.01],
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

    s.stop_logging()
    robot.robot_logger.plot(
        plot_selection=RobotPlotFlags.END_EFFECTOR | RobotPlotFlags.JOINTS
    )
