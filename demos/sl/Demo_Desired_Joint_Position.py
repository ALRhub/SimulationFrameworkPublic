import numpy as np

from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository

if __name__ == "__main__":
    REAL_ROBOT = True

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

    duration = 4
    robot_init_q = robot.current_j_pos

    s.start_logging()  # log the data for plotting

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
    robot.set_gripper_width = 0.04
    robot.gotoJointPosition(des_q, duration=duration)

    """Close the fingers."""
    duration = 3
    robot.close_fingers(duration=0.5)

    """Go back to inital position, let the fingers closed."""
    duration = 4
    robot.gotoJointPosition(robot_init_q, duration=duration)

    s.stop_logging()

    rMSE = np.sqrt(
        np.mean(
            (robot.robot_logger.joint_pos - robot.robot_logger.des_joint_pos) ** 2,
            axis=0,
        )
    )
    print("Tracking Error: ", np.mean(rMSE))

    robot.robot_logger.plot(plot_selection=RobotPlotFlags.JOINTS)
