import os

import numpy as np

from alr_sim.controllers.Controller import (
    ModelBasedFeedforwardController,
)
from alr_sim.controllers.TrajectoryTracking import CartPosQuatTrajectoryTracker
from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository

if __name__ == "__main__":
    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")

    scene = sim_factory.create_scene()
    robot = sim_factory.create_robot(scene)
    cartesianTracking = CartPosQuatTrajectoryTracker(robot.dt)
    scene.start()

    # load the trajectory you want to follow
    path2file = os.path.join(os.path.dirname(__file__), "des_task_traj.npy")
    des_task_trajectory = np.load(path2file)

    # robot.jointTrackingController = JointPDController()
    robot.jointTrackingController = ModelBasedFeedforwardController()

    scene.start_logging()
    cartesianTracking.setTrajectory(des_task_trajectory)
    cartesianTracking.executeController(robot, des_task_trajectory.shape[0] * 0.001)
    scene.stop_logging()

    rMSE = np.sqrt(
        np.mean(
            (robot.robot_logger.cart_pos - robot.robot_logger.des_c_pos) ** 2, axis=0
        )
    )
    print("Tracking Error: ", np.mean(rMSE))

    robot.robot_logger.plot(plot_selection=RobotPlotFlags.JOINTS)
