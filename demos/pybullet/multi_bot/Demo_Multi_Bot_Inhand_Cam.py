import matplotlib.pyplot as plt
from alr_sim.sims.SimFactory import SimRepository

if __name__ == "__main__":
    # Setup the scene
    sim_factory = SimRepository.get_factory("pybullet")

    scene = sim_factory.create_scene()
    robot = sim_factory.create_robot(
        scene, base_position=[-0.1, -0.5, 0.0], base_orientation=[0.7, 0.0, 0.0, 0.7]
    )
    robot2 = sim_factory.create_robot(
        scene, base_position=[0.4, 0.5, 0.0], base_orientation=[0.7, 0.0, 0.0, -0.7]
    )
    scene.start()

    home_position = robot.current_c_pos

    scene.robots.gotoCartPositionAndQuat(
        desiredPos=[[0.6, -0.20, 0.02]] * len(scene.robots),
        desiredQuat=[[0.009, 0.72, -0.67, -0.014]] * len(scene.robots),
        global_coord=False,
    )

    # This program tends to freeze. See also https://pybullet.org/Bullet/phpBB3/viewtopic.php?p=43935&hilit=camera#p43935
    plt.subplot(221)
    plt.imshow(robot.inhand_cam.get_segmentation(depth=False))
    plt.subplot(222)
    plt.imshow(robot2.inhand_cam.get_segmentation(depth=False))

    scene.robots.gotoCartPositionAndQuat(
        desiredPos=[home_position] * len(scene.robots),
        desiredQuat=[[1, 0, 1, 0]] * len(scene.robots),
        global_coord=False,
    )
    plt.subplot(223)
    plt.imshow(robot.inhand_cam.get_segmentation(depth=False))
    plt.subplot(224)
    plt.imshow(robot2.inhand_cam.get_segmentation(depth=False))
    plt.show()
