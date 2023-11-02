import numpy as np

from alr_sim.controllers.Controller import (
    JointPDController,
)
from alr_sim.core.logger import ObjectLogger
from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box

if __name__ == "__main__":
    box = Box(
        name="box",
        init_pos=[0.625, -0.2, 0.15],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 0, 0, 1],
        size=[0.02, 0.04, 0.04],
    )
    object_list = [box]

    # Setup the scene
    sim_factory = SimRepository.get_factory("pybullet")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)
    robot.use_inv_dyn = False

    scene.start()

    robot.jointTrackingController = JointPDController()

    duration = 4
    robot_init_q = robot.current_j_pos

    objectLogger = ObjectLogger(scene, scene.get_object("box"))
    scene.add_logger(objectLogger)
    scene.start_logging()  # log the data for plotting

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
    robot.close_fingers(duration=0.5)

    """Go back to inital position, let the fingers closed."""
    duration = 4
    robot.gotoJointPosition(robot_init_q, duration=duration)

    scene.stop_logging()

    rMSE = np.sqrt(
        np.mean(
            (robot.robot_logger.joint_pos - robot.robot_logger.des_joint_pos) ** 2,
            axis=0,
        )
    )
    print("Tracking Error: ", np.mean(rMSE))

    robot.robot_logger.plot(
        plot_selection=RobotPlotFlags.JOINTS | RobotPlotFlags.GRIPPER
    )
    objectLogger.plot()

    import matplotlib.pyplot as plt

    plt.show()
