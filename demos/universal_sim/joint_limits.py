import numpy as np

from alr_sim.sims.SimFactory import SimRepository

if __name__ == "__main__":
    """
    Drive sequentelly to each joint's limit.
    """
    sim_factory = SimRepository.get_factory("mujoco")

    ### Create Robot and Scene. REQUIRED!
    robot = sim_factory.create_robot()
    scene = sim_factory.create_scene(robot)
    scene.start()

    ### Control Commands:
    robot.receiveState()
    init_joint_pos = robot.current_j_pos

    duration = 8.0

    joint_limits = np.array(
        [
            [-166, 166],
            [-101, 101],
            [-166, 166],
            [-176, -4],
            [-166, 166],
            [-1, 215],
            [-166, 166],
        ]
    )

    joint_limits = joint_limits / 180.0 * np.pi / 2.0

    for joint_index in range(7):
        for limit in joint_limits[joint_index]:
            robot.gotoJointPosition(init_joint_pos, duration=duration)
            limit_pos = init_joint_pos.copy()
            limit_pos[joint_index] = limit
            robot.gotoJointPosition(limit_pos, duration=duration)
