import numpy as np

from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.mujoco import MujocoRobot
from alr_sim.sims.mujoco import MujocoScene as Scene

if __name__ == "__main__":
    object_list = []
    # Setup the scene
    scene = Scene(object_list=object_list)

    mj_Robot = MujocoRobot(scene, gravity_comp=True, num_DoF=7)

    scene.start_logging()  # log the data for plotting

    des_torques = np.load("des_torques.npy")
    for t in range(des_torques.shape[0]):
        mj_Robot.executeTorqueCtrlTimeStep(des_torques[t, :], timeSteps=1)

    scene.stop_logging()
    mj_Robot.robot_logger.plot(plot_selection=RobotPlotFlags.COMMAND)

    # saved_torques = np.load('des_torques.npy')
    # import matplotlib.pyplot as plt
    # plt.figure()
    # for i in range(saved_torques.shape[1]):
    #     plt.subplot(7,1, i+1)
    #     plt.plot(mj_Robot.robot_logger.command[:, i], 'blue')
    #     plt.plot(saved_torques[:, i], 'red')
