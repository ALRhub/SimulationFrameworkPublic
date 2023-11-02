import matplotlib.pyplot as plt

from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Box

if __name__ == "__main__":

    # Warning - The script works but Mujoco visualization breaks on my system after the first segmentation call

    box1 = Box(
        name="box1",
        init_pos=[0.5, -0.2, 0.35],
        init_quat=[0, 1, 0, 0],
        rgba=[0.1, 0.25, 0.3, 1],
    )
    box2 = Box(
        name="box2",
        init_pos=[0.6, -0.1, 0.35],
        init_quat=[0, 1, 0, 0],
        rgba=[0.2, 0.3, 0.7, 1],
    )
    box3 = Box(
        name="box3",
        init_pos=[0.4, -0.1, 0.35],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 0, 0, 1],
    )
    box4 = Box(
        name="box4",
        init_pos=[0.6, -0.0, 0.35],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 0, 0, 1],
    )
    box5 = Box(
        name="box5",
        init_pos=[0.6, 0.1, 0.35],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 1, 1, 1],
    )
    box6 = Box(
        name="box6",
        init_pos=[0.6, 0.2, 0.35],
        init_quat=[0, 1, 0, 0],
        rgba=[1, 0, 0, 1],
    )

    table = Box(
        name="table0",
        init_pos=[0.5, 0.0, 0.2],
        init_quat=[0, 1, 0, 0],
        size=[0.25, 0.35, 0.2],
        static=True,
    )

    object_list = [box1, box2, box3, box4, box5, box6, table]

    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")
    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)
    scene.start()

    duration = 2
    robot.set_desired_gripper_width(0.0)

    home_position = robot.current_c_pos.copy()
    home_orientation = robot.current_c_quat.copy()

    robot.gotoCartPositionAndQuat(
        [0.5, -0.2, 0.6 - 0.1], [0, 1, 0, 0], duration=duration
    )
    robot.set_desired_gripper_width(0.04)

    cam_name = "rgbd_cage"

    segmentation_data = scene.get_object(cam_name).get_segmentation(depth=False)
    plt.subplot(141)
    plt.imshow(scene.get_object(cam_name).get_segmentation(depth=False))

    robot.gotoCartPositionAndQuat(
        [0.5, -0.2, 0.52 - 0.1], [0, 1, 0, 0], duration=duration
    )
    robot.close_fingers()

    plt.subplot(142)
    plt.imshow(scene.get_cage_cam().get_segmentation(depth=False))

    robot.gotoCartPositionAndQuat(home_position, home_orientation, duration=duration)
    robot.gotoCartPositionAndQuat(
        [0.5, 0.2, 0.6 - 0.1], [0, 1, 0, 0], duration=duration
    )
    robot.set_desired_gripper_width(0.04)

    plt.subplot(143)
    plt.imshow(scene.get_cage_cam().get_segmentation(depth=False))

    robot.gotoCartPositionAndQuat(
        [0.6, -0.1, 0.6 - 0.1], [0, 1, 0, 0], duration=duration
    )
    robot.gotoCartPositionAndQuat(
        [0.6, -0.1, 0.52 - 0.1], [0, 1, 0, 0], duration=duration
    )
    robot.close_fingers()
    robot.gotoCartPositionAndQuat(home_position, home_orientation, duration=duration)
    robot.gotoCartPositionAndQuat(
        [0.5, 0.2, 0.6 - 0.1], [0, 1, 0, 0], duration=duration
    )
    robot.set_desired_gripper_width(0.04)

    plt.subplot(144)
    plt.imshow(scene.get_cage_cam().get_segmentation(depth=False))

    robot.gotoCartPositionAndQuat(
        [0.4, -0.1, 0.6 - 0.1], [0, 1, 0, 0], duration=duration
    )
    robot.gotoCartPositionAndQuat(
        [0.4, -0.1, 0.52 - 0.1], [0, 1, 0, 0], duration=duration
    )
    robot.close_fingers()
    robot.gotoCartPositionAndQuat(home_position, home_orientation, duration=duration)
    robot.gotoCartPositionAndQuat(
        [0.5, 0.2, 0.65 - 0.1], [0, 1, 0, 0], duration=duration
    )
    robot.set_desired_gripper_width(0.04)
    robot.gotoCartPositionAndQuat(home_position, home_orientation, duration=duration)

    plt.show()
