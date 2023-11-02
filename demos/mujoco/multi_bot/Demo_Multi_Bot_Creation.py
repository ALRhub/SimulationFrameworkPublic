import numpy as np

from alr_sim.core.logger import RobotPlotFlags, ObjectLogger
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
    sim_factory = SimRepository.get_factory("mujoco")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)
    robot2 = sim_factory.create_robot(scene, base_position=[2.0, 0.0, 0.0])
    robot.use_inv_dyn = False

    scene.start()
    scene.start_logging(plot_selection=RobotPlotFlags.JOINT_POS)

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
    robot.open_fingers()
    robot.gotoJointPosition(des_q, duration=duration)
    robot.close_fingers()
    robot.gotoJointPosition(robot_init_q)
    robot2.gotoJointPosition(des_q, duration=duration)
    robot.open_fingers()
    robot.gotoJointPosition(des_q)
    robot2.wait(2)
    scene.stop_logging()
    robot.robot_logger.plot()
    robot2.robot_logger.plot()
