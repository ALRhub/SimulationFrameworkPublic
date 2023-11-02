import numpy as np

from alr_sim.controllers.Controller import (
    ModelBasedFeedforwardController,
)
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

    # load the trajectory you want to follow
    path2file = "./des_joint_traj.npy"
    des_joint_trajectory = np.load(path2file)

    # robot.jointTrackingController = JointPDController()
    robot.jointTrackingController = ModelBasedFeedforwardController()

    s.start_logging()
    robot.follow_JointTraj(des_joint_trajectory)

    s.stop_logging()

    rMSE = np.sqrt(
        np.mean(
            (robot.robot_logger.joint_pos - robot.robot_logger.des_joint_pos) ** 2,
            axis=0,
        )
    )
    print("Tracking Error: ", np.mean(rMSE))

    robot.robot_logger.plot(plot_selection=RobotPlotFlags.JOINTS)
