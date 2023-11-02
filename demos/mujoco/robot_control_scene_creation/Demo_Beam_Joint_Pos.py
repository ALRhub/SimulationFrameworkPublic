import numpy as np

from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository

from alr_sim.controllers.Controller import (
    ModelBasedFeedforwardController,
)


if __name__ == "__main__":
    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")

    s = sim_factory.create_scene()
    robot = sim_factory.create_robot(s)

    s.start()

    robot.jointTrackingController = ModelBasedFeedforwardController()

    robot_init_q = robot.current_j_pos

    des_q = np.array(
        [
            -0.17372284,
            0.74377287,
            -0.15055875,
            -1.8271288,
            0.17003154,
            2.52458572,
            1.85687575,
        ]
    )

    robot.startLogging(
        duration=11,  # log for 11 seconds
        log_interval=0.01,  # ... at an interval of 0.01 seconds
        plot_selection=RobotPlotFlags.JOINTS,
    )

    robot.gotoJointPosition(des_q)

    robot.beam_to_joint_pos(robot_init_q)
    robot.gotoCartPositionAndQuat([0.3, 0.0, 0.5], [0, 1, 0, 0])

    robot.beam_to_joint_pos(robot_init_q)
    robot.gotoJointPosition(des_q)

    robot.stopLogging()

    robot.robot_logger.plot()
