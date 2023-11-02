import os
import numpy as np

from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository

if __name__ == "__main__":
    # Setup the scene
    sim_factory = SimRepository.get_factory("pybullet")

    scene = sim_factory.create_scene()
    robot = sim_factory.create_robot(
        scene, base_position=[0.0, -0.5, 0.0], base_orientation=[1.0, 0.0, 0.0, -0.1]
    )
    robot2 = sim_factory.create_robot(
        scene, base_position=[0.0, 0.5, 0.0], base_orientation=[1.0, 0.0, 0.0, 0.1]
    )

    scene.start()

    duration = 4
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

    ### GOTO JOINTPOS
    scene.start_logging(plot_selection=RobotPlotFlags.JOINT_POS)
    robot.gotoJointPosition(des_q, block=False)
    robot2.gotoJointPosition(des_q, block=True)

    scene.stop_logging()
    robot.robot_logger.plot(block=False)
    robot2.robot_logger.plot()

    ### FOLLOW JOINT TRAJ
    scene.start_logging(plot_selection=RobotPlotFlags.JOINT_POS)
    path2file = os.path.join(
        os.path.dirname(__file__),
        "..",
        "robot_control_scene_creation",
        "des_joint_traj.npy",
    )
    des_joint_trajectory = np.load(path2file)

    robot.follow_JointTraj(des_joint_trajectory, block=False)
    robot2.follow_JointTraj(des_joint_trajectory)
    scene.stop_logging()
    robot.robot_logger.plot(block=False)
    robot2.robot_logger.plot()

    ### GOTO CART POS QUAT IMPEDANCE
    scene.start_logging(plot_selection=RobotPlotFlags.CART_POS)
    robot.gotoCartPositionAndQuat_ImpedanceCtrl(
        [0.15, -0.1, 0.15], [0, 0.7, 0.7, 0], block=False
    )
    robot2.gotoCartPositionAndQuat_ImpedanceCtrl([0.15, 0.1, 0.15], [0, 0.7, -0.7, 0])

    scene.stop_logging()
    robot.robot_logger.plot(block=False)
    robot2.robot_logger.plot()

    ### GOTO CART POS QUAT CONTROLLER
    scene.reset()
    scene.start_logging(plot_selection=RobotPlotFlags.CART_POS)
    robot.gotoCartPositionAndQuat([0.15, -0.1, 0.15], [0, 0.7, 0.7, 0], block=False)
    robot2.gotoCartPositionAndQuat([0.15, 0.1, 0.15], [0, 0.7, -0.7, 0])
    scene.stop_logging()
    robot.robot_logger.plot(block=False)
    robot2.robot_logger.plot()
