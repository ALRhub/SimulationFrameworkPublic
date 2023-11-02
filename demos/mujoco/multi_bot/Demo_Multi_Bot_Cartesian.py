import numpy as np

from alr_sim.core.logger import RobotPlotFlags, ObjectLogger
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box

if __name__ == "__main__":
    box = Box(
        name="box",
        init_pos=[0.15, 0, 0.15],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 0, 0, 1],
        size=[0.02, 0.04, 0.04],
    )
    object_list = [box]

    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(
        scene, base_position=[0.0, -0.5, 0.2], base_orientation=[1.0, 0.0, 0.3, 0.0]
    )
    robot2 = sim_factory.create_robot(
        scene, base_position=[0.0, 0.5, 0.2], base_orientation=[1.0, 0.0, 0.3, 0.0]
    )
    robot.use_inv_dyn = False

    scene.start()
    scene.start_logging(plot_selection=RobotPlotFlags.END_EFFECTOR_GLOBAL)

    robot_init_q = robot.current_j_pos

    robot.open_fingers()
    robot.gotoCartPositionAndQuat(box.init_pos, [0, 0.7, 0.7, 0])
    robot.close_fingers()
    robot.gotoJointPosition(robot_init_q)
    robot2.gotoCartPositionAndQuat_ImpedanceCtrl(
        box.init_pos,
        desiredQuat=[0.0, 0.7, -0.7, 0],
    )
    scene.stop_logging()
    robot.robot_logger.plot()
    robot2.robot_logger.plot()
