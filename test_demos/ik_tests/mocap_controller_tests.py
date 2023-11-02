from alr_sim.sims.mujoco import MujocoMocapRobot
from alr_sim.sims.mujoco import MujocoScene as Scene
from alr_sim.sims.mujoco import mujoco_controllers as mj_ctrl

import numpy as np

if __name__ == "__main__":

    object_list = []

    scene = Scene(
        object_list=object_list, control=mj_ctrl.MocapControl()
    )  # if we want to do mocap control
    # scene = Scene(object_list=object_list)                        # ik control is default

    mj_Robot = MujocoMocapRobot(scene, gravity_comp=True, num_DoF=7)

    dur = (
        0.1  # you can specify how long a trajectory can be executed witht the duration
    )

    poses = np.load("poselog.npy", allow_pickle=True)
    print(poses)
    loss = 0
    for p in poses:
        # mj_Robot.gotoCartPositionAndQuat_ImpedanceCtrl(p[0], p[1], duration=dur)
        mj_Robot.gotoCartPositionAndQuat(p[0], p[1], duration=dur)
        loss += np.linalg.norm(mj_Robot.current_c_pos - p[0])
        loss += np.linalg.norm(mj_Robot.current_c_quat - p[1])
        loss += 0  # Endeffector velocity?
        print(loss)
