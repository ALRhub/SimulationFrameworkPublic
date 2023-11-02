from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box
from alr_sim.controllers.IKControllers import (
    CartPosQuatImpedenceJacTransposeController,
    CartPosQuatImpedenceController,
)
import numpy as np

if __name__ == "__main__":
    object_list = []
    duration = 4

    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)
    robot.use_inv_dyn = False

    robot.cartesianPosQuatTrackingController = CartPosQuatImpedenceController()
    scene.start()

    home_position = np.array([5.50899712e-01, -1.03382391e-08, 6.5e-01])
    scene.start_logging()  # log the data for plotting

    robot.set_gripper_width = 0.04

    #### front left bottom corner
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, -0.25, 0.15],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, -0.25, 0.15],
        desiredQuat=[0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, -0.25, 0.15],
        desiredQuat=[-0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    #### front right bottom corner
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, 0.25, 0.15],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, 0.25, 0.15],
        desiredQuat=[0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, 0.25, 0.15],
        desiredQuat=[-0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    #### front right top corner
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, 0.25, 0.65],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, 0.25, 0.65],
        desiredQuat=[0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, 0.25, 0.65],
        desiredQuat=[-0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    #### front left top corner
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, -0.25, 0.65],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, -0.25, 0.65],
        desiredQuat=[0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.6, -0.25, 0.65],
        desiredQuat=[-0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    #### back left bottom corner
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, -0.25, 0.15],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, -0.25, 0.15],
        desiredQuat=[0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, -0.25, 0.15],
        desiredQuat=[-0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    #### back right bottom corner
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, 0.25, 0.15],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, 0.25, 0.15],
        desiredQuat=[0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, 0.25, 0.15],
        desiredQuat=[-0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    #### back right top corner
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, 0.25, 0.45],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, 0.25, 0.45],
        desiredQuat=[0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, 0.25, 0.45],
        desiredQuat=[-0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    #### back left top corner
    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, -0.25, 0.45],
        desiredQuat=[0.009, 0.72, -0.67, -0.014],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, -0.25, 0.45],
        desiredQuat=[0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    robot.gotoCartPositionAndQuat(
        desiredPos=[0.35, -0.25, 0.45],
        desiredQuat=[-0.7071, 0.7071, 0, 0],
        duration=duration,
    )

    """Go back to inital position, let the fingers closed."""
    duration = 4

    robot.gotoCartPositionAndQuat(
        desiredPos=home_position, desiredQuat=[0, 1, 0, 0], duration=duration
    )

    scene.stop_logging()
    robot.robot_logger.plot(plot_selection=RobotPlotFlags.END_EFFECTOR)
