import numpy as np

from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.mujoco import MujocoRobot
from alr_sim.sims.mujoco import MujocoScene as Scene

if __name__ == "__main__":
    object_list = []
    # Setup the scene
    scene = Scene(object_list=object_list)

    mj_Robot = MujocoRobot(scene, gravity_comp=True, num_DoF=7)

    robot_init_q = mj_Robot.current_j_pos
    scene.start_logging()  # log the data for plotting

    des_joint_vel = np.load("des_joint_vel_traj.npy")

    for t in range(des_joint_vel.shape[0]):
        mj_Robot.executeJointVelCtrlTimeStep(des_joint_vel[t, :], timeSteps=1)

    scene.stop_logging()
    mj_Robot.robot_logger.plot(plot_selection=RobotPlotFlags.JOINTS)
