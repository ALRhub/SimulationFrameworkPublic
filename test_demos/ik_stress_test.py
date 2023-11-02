from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box
from alr_sim.controllers.IKControllers import CartPosQuatImpedenceJacTransposeController
import numpy as np

if __name__ == "__main__":
    object_list = []
    duration = 4

    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)
    robot.use_inv_dyn = False

    # robot.cartesianPosQuatTrackingController.neglect_dynamics = True
    scene.start()

    home_position = np.array([5.50899712e-01, -1.03382391e-08, 6.5e-01])

    robot.set_gripper_width = 0.04

    desiredPos = []
    desiredQuat = []

    desiredPos.append([0.6, -0.25, 0.15])
    desiredQuat.append([0.009, 0.72, -0.67, -0.014])

    desiredPos.append([0.6, -0.25, 0.15])
    desiredQuat.append([0.7071, 0.7071, 0, 0])

    desiredPos.append([0.6, -0.25, 0.15])
    desiredQuat.append([-0.7071, 0.7071, 0, 0])

    #### front right bottom corner
    desiredPos.append([0.6, 0.25, 0.15])
    desiredQuat.append([0.009, 0.72, -0.67, -0.014])

    desiredPos.append([0.6, 0.25, 0.15])
    desiredQuat.append([0.7071, 0.7071, 0, 0])

    desiredPos.append([0.6, 0.25, 0.15])
    desiredQuat.append([-0.7071, 0.7071, 0, 0])

    #### front right top corner
    desiredPos.append([0.6, 0.25, 0.65])
    desiredQuat.append([0.009, 0.72, -0.67, -0.014])

    desiredPos.append([0.6, 0.25, 0.65])
    desiredQuat.append([0.7071, 0.7071, 0, 0])

    desiredPos.append([0.6, 0.25, 0.65])
    desiredQuat.append([-0.7071, 0.7071, 0, 0])

    #### front left top corner
    desiredPos.append([0.6, -0.25, 0.65])
    desiredQuat.append([0.009, 0.72, -0.67, -0.014])

    desiredPos.append([0.6, -0.25, 0.65])
    desiredQuat.append([0.7071, 0.7071, 0, 0])

    desiredPos.append([0.6, -0.25, 0.65])
    desiredQuat.append([-0.7071, 0.7071, 0, 0])

    #### back left bottom corner
    desiredPos.append([0.35, -0.25, 0.65])
    desiredQuat.append([0.009, 0.72, -0.67, -0.014])

    desiredPos.append([0.35, -0.25, 0.65])
    desiredQuat.append([0.7071, 0.7071, 0, 0])

    desiredPos.append([0.35, -0.25, 0.65])
    desiredQuat.append([-0.7071, 0.7071, 0, 0])

    #### back right bottom corner

    desiredPos.append([0.35, 0.25, 0.65])
    desiredQuat.append([0.009, 0.72, -0.67, -0.014])

    desiredPos.append([0.35, 0.25, 0.65])
    desiredQuat.append([0.7071, 0.7071, 0, 0])

    desiredPos.append([0.35, 0.25, 0.65])
    desiredQuat.append([-0.7071, 0.7071, 0, 0])

    #### back right top corner

    desiredPos.append([0.35, 0.25, 0.45])
    desiredQuat.append([0.009, 0.72, -0.67, -0.014])

    desiredPos.append([0.35, 0.25, 0.45])
    desiredQuat.append([0.7071, 0.7071, 0, 0])

    desiredPos.append([0.35, 0.25, 0.45])
    desiredQuat.append([-0.7071, 0.7071, 0, 0])

    #### back left top corner
    desiredPos.append([0.35, -0.25, 0.45])
    desiredQuat.append([0.009, 0.72, -0.67, -0.014])

    desiredPos.append([0.35, -0.25, 0.45])
    desiredQuat.append([0.7071, 0.7071, 0, 0])

    desiredPos.append([0.35, -0.25, 0.45])
    desiredQuat.append([-0.7071, 0.7071, 0, 0])

    error_i_pos = np.zeros(len(desiredPos))
    error_i_quat = np.zeros(len(desiredPos))

    robot.clip_actions = True

    scene.start_logging()  # log the data for plotting
    for i in range(len(desiredQuat)):

        robot.gotoCartPositionAndQuat_ImpedanceCtrl(
            desiredPos=desiredPos[i],
            desiredQuat=desiredQuat[i],
            duration=duration,
        )

        # error_i_pos[i] = np.sqrt(
        #     np.mean((robot.robot_logger.cart_pos - robot.robot_logger.des_c_pos) ** 2)
        # )
        # error_i_quat[i] = np.sqrt(
        #     np.mean((robot.robot_logger.cart_quat - robot.robot_logger.des_quat) ** 2)
        # )
        # print(
        #     "%d: Pos error: %f, Quat error: %f\n" % (i, error_i_pos[i], error_i_quat[i])
        # )
    scene.stop_logging()

    print("Mean errors: %f %f\n" % (np.mean(error_i_pos), np.mean(error_i_quat)))
    """Go back to inital position, let the fingers closed."""
    robot.robot_logger.plot(
        plot_selection=RobotPlotFlags.END_EFFECTOR
        | RobotPlotFlags.JOINTS
        | RobotPlotFlags.TORQUES
    )
