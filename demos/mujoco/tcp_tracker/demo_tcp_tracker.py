from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from demos.mujoco.tcp_tracker.tcp_marker import TcpOffsetTracker, MatrixTracker

if __name__ == "__main__":
    box = TcpOffsetTracker()
    box2 = MatrixTracker()
    object_list = [box, box2]
    duration = 4

    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)
    scene.start()

    # Callback hack to sync tracker with TCP
    def callback(s, r):
        box.sync(s, r)
        box2.sync(s, r)

    scene.register_callback(callback, s=scene, r=robot)

    home_position = robot.current_c_pos
    scene.start_logging(
        plot_selection=RobotPlotFlags.END_EFFECTOR
    )  # log the data for plotting

    robot.set_desired_gripper_width(0.04)
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, -0.20, 0.02],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )
    """Close the fingers."""
    robot.close_fingers()
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, -0.20, 0.02],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    """Go back to inital position, let the fingers closed."""
    robot.gotoCartPositionAndQuat(
        desiredPos=home_position, desiredQuat=[1, 0, 1, 0], duration=duration
    )

    scene.stop_logging()
    robot.robot_logger.plot()
