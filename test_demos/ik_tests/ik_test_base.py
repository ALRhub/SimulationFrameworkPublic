import numpy as np
from alr_sim.sims.mujoco import MujocoRobot
from alr_sim.sims.mujoco import MujocoScene as Scene


class IKRobot(MujocoRobot):
    def __init__(self, scene, move_dur=1.0):
        super(IKRobot, self).__init__(scene)

        self.move_dur = move_dur
        self.move_func = self.gotoCartPositionAndQuat

    def move_to(self, target_pos, target_quat):
        print(target_pos, target_quat)
        self.move_func(target_pos, target_quat, self.move_dur)


def create_test(move_dur=1.0, render=True):
    scene = Scene(render=render)
    robot = IKRobot(scene, move_dur=move_dur)
    return scene, robot


DEFAULT_TRAJECTORIES = [
    [[0.5, 0.0, 0.6], [0.0, 1.0, 0.0, 0.0]],
    [[0.5, 0.0, 0.1], [0.0, 0.5, 0.5, 0.0]],
    [[0.5, 0.5, 0.8], [0.0, 1.0, 0.0, 0.0]],
    [[0.0, 0.5, 0.2], [0.0, 0.0, 0.5, 0.5]],
    [[-0.5, 0.0, 0.8], [0.0, 0.0, 1.0, 0.0]],
    [[0.1, -0.8, 0.3], [0.0, 1.0, 0.0, 0.0]],
    [[0.5, 0.0, 0.6], [1.0, 0.0, 0.0, 0.0]],
    [[0.5, 0.0, 0.6], [0.0, 1.0, 0.0, 0.0]],
]


def execute_test(robot: IKRobot, cart_trajectories=None):
    loss = 0

    if cart_trajectories is None:
        cart_trajectories = DEFAULT_TRAJECTORIES

    for traj in cart_trajectories:
        target_pos, target_quat = traj[0], traj[1]
        robot.move_to(target_pos, target_quat)

        loss += np.linalg.norm(robot.current_c_pos - target_pos)
        loss += np.linalg.norm(robot.current_c_quat - target_quat)
        print("EPISODE LOSS", loss)

    print("TOTAL LOSS:", loss)
    return loss
