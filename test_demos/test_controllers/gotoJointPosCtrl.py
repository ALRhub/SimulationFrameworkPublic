import numpy as np

from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.mujoco import MujocoRobot
from alr_sim.sims.mujoco import MujocoScene as Scene
from alr_sim.sims.mujoco import MujocoPrimitiveObject

if __name__ == "__main__":
    box = MujocoPrimitiveObject(
        obj_pos=[0.6, -0.2, 0.15],
        obj_name="box",
        diaginertia=[0.01, 0.01, 0.01],
        geom_rgba=[1, 0, 0, 1],
    )
    object_list = [box]
    duration = 4
    # Setup the scene
    scene = Scene(object_list=object_list)

    mj_Robot = MujocoRobot(scene, gravity_comp=True, num_DoF=7)

    robot_init_q = mj_Robot.current_j_pos
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
    mj_Robot.set_gripper_width = 0.04
    mj_Robot.gotoJointPosition(des_q, duration=duration)

    """Close the fingers."""
    duration = 3
    mj_Robot.set_gripper_width = 0.0
    mj_Robot.gotoJointPosition(des_q, duration=3)

    """Go back to inital position, let the fingers closed."""
    duration = 4
    mj_Robot.gotoJointPosition(robot_init_q, duration=duration)

    scene.stop_logging()
    mj_Robot.robot_logger.plot(plot_selection=RobotPlotFlags.JOINTS)
