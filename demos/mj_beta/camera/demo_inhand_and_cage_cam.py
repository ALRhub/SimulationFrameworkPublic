from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box
from matplotlib import pyplot as plt

if __name__ == "__main__":
    box1 = Box(
        name="box1",
        init_pos=[0.3, -0.2, 0],
        init_quat=[0, 1, 0, 0],
        rgba=[0.1, 0.25, 0.3, 1],
    )
    box2 = Box(
        name="box2",
        init_pos=[0.3, 0.2, 0],
        init_quat=[0, 1, 0, 0],
        rgba=[0.2, 0.3, 0.7, 1],
    )

    object_list = [box1, box2]

    # Setup the scene
    sim_factory = SimRepository.get_factory("mj_beta")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)

    # Setting custom parameters for this camera
    # MUST BE DONE BEFORE THE scene.start()
    robot.inhand_cam.set_cam_params(width=640, height=480, fovy=60)

    scene.start()

    robot.gotoCartPositionAndQuat(desiredPos=[0.3, 0, 0.6], desiredQuat=[0, 1, 0, 0])
    robot.wait(2)

    # Gathering all possible images from the inhand camera
    seg_img = robot.inhand_cam.get_segmentation(depth=False)
    rgb_img, depth_img = robot.inhand_cam.get_image(depth=True)

    # Visualizing these images
    plt.figure("InHand Segmentation Image")
    plt.imshow(seg_img)

    plt.figure("InHand RGB Image")
    plt.imshow(rgb_img)

    plt.figure("InHand Depth Image")
    plt.imshow(depth_img)
    plt.show()

    # Gathering all possible images from the cage camera
    seg_img = scene.cage_cam.get_segmentation(depth=False)
    rgb_img, depth_img = scene.cage_cam.get_image(depth=True)

    # Visualizing these images
    plt.figure("Cage Segmentation Image")
    plt.imshow(seg_img)

    plt.figure("Cage RGB Image")
    plt.imshow(rgb_img)

    plt.figure("Cage Depth Image")
    plt.imshow(depth_img)
    plt.show()
