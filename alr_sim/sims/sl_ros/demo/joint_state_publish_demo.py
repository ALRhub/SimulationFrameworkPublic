import time

from alr_sim.sims import SimFactory

# from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.sl_ros.RosNode import RosNode
from alr_sim.sims.sl_ros.RosPublisher import *
from alr_sim.sims.universal_sim.PrimitiveObjects import Box

if __name__ == "__main__":
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

    node = RosNode("SimPanda", tick_step=1, components=None)

    # Setup the scene
    sim_factory = SimRepository.get_factory("mujoco")
    s = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(
        s,
        step_callback=[
            node.run,
        ],
    )
    s.start()

    q = JointStatePublisher(robot, tick=100)
    p = CartesianStatePublisher(robot, tick=100)
    tf = TFBroadcaster(robot, s.get_object("rgbd"), tick=100)
    pcl = PclPublisher(robot, s.get_object("rgbd"), tick=1000)
    img = ImgPublisher(robot, s.get_object("rgbd"), tick=100)
    depth = DepthImgPublisher(robot, s.get_object("rgbd"), tick=100)

    # p = PclFloatPublisher(s.get_object("rgbd"), tick=100)
    # node.add_components([tf, pcl])
    node.add_components([q, p, tf, pcl, img, depth])

    # you can specify how long a trajectory can be executed witht the duration
    duration = 2

    # s.start_logging()  # this will start logging robots internal state
    robot.set_gripper_width = 0.0  # we set the gripper to clos at the beginning

    home_position = robot.current_c_pos.copy()  # store home position
    home_orientation = robot.current_c_quat.copy()  # store initial orientation

    # execute the pick and place movements
    while True:
        #
        # robot.gotoCartPositionAndQuat([0.5, 0, 0.6],
        #                               [0, 1, 1, 0],
        #                               duration=duration)

        robot.gotoCartPositionAndQuat([0.5, -0.2, 0.6], [0, 1, 1, 0], duration=duration)
        robot.gotoCartPositionAndQuat([0.5, 0.2, 0.6], [0, 1, 0, 0], duration=duration)

    # s.stop_logging()
    # robot.robot_logger.plot(RobotPlotFlags.JOINTS | RobotPlotFlags.END_EFFECTOR)
