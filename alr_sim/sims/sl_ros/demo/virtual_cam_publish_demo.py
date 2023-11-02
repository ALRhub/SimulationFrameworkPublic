from alr_sim.sims import SimFactory
from alr_sim.sims.sl_ros.RosPublisher import PclFloatPublisher
from alr_sim.sims.universal_sim.PrimitiveObjects import Box

if __name__ == "__main__":
    object_list = [Box(None, [0.5, 0, 0.2], [0, 1, 0, 0], rgba=[1, 0, 0, 1])]

    sim_factory = SimFactory.SimRepository.get_factory("mujoco")
    robot = sim_factory.create_robot()
    s = sim_factory.create_scene(robot, object_list=object_list)
    s.start()

    # robot.gotoCartPositionAndQuat([1, 0.5, 1], [0, 1, 0, 0])

    p = PclFloatPublisher(s.get_object("rgbd"))
    p.run()
