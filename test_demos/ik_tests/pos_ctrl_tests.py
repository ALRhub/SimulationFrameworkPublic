from alr_sim.sims.mujoco import MujocoRobot
from alr_sim.sims.mujoco import MujocoScene as Scene
from alr_sim.sims.mujoco import mujoco_controllers as mj_ctrl

import numpy as np

if __name__ == "__main__":

    object_list = []

    scene = Scene(
        object_list=object_list, control=mj_ctrl.PositionControl()
    )  # if we want to do mocap control
    # scene = Scene(object_list=object_list)                        # ik control is default

    mj_Robot = MujocoRobot(scene, gravity_comp=True, num_DoF=7)
    scene.start_logging()
    dur = (
        1.5  # you can specify how long a trajectory can be executed witht the duration
    )

    poses = np.load("poselog.npy", allow_pickle=True)
    print(poses)
    loss = 0
    for p in poses:
        # mj_Robot.gotoCartPositionAndQuat_ImpedanceCtrl(p[0], p[1], duration=dur)
        mj_Robot.gotoJointPosition(p[2], duration=dur)
        loss += np.linalg.norm(mj_Robot.current_j_pos - p[2])
        print(loss)
    scene.stop_logging()
    mj_Robot.robot_logger.plot()
