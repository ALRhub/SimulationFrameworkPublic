import numpy as np

from alr_sim.sims.SimFactory import SimRepository

if __name__ == "__main__":
    """
    put the robot in the transport pose.
    """
    sim_factory = SimRepository.get_factory("pybullet")

    ### Create Robot and Scene. REQUIRED!
    scene = sim_factory.create_scene()
    robot = sim_factory.create_robot(scene)
    scene.start()

    duration = 8.0

    transport_pose_angles = np.array([0, -32.08, 0, -170.17, 0, 0, 45])

    transport_pose_rads = transport_pose_angles / 180.0 * np.pi / 2.0
    robot.gotoJointPosition(transport_pose_rads)
