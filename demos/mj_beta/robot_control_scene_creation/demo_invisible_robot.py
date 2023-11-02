from alr_sim.sims.SimFactory import SimRepository
from alr_sim.utils import sim_path

if __name__ == "__main__":
    # Setup the scene
    sim_factory = SimRepository.get_factory("mj_beta")

    scene = sim_factory.create_scene()
    # robot = sim_factory.create_robot(scene)
    robot = sim_factory.create_robot(scene)

    # A new visual XML may also be located completely outside of the Sim Framework
    robot._full_xml_path = sim_path.sim_framework_path(
        "./models/mj/robot/panda_invisible.xml"
    )
    scene.start()

    robot.gotoCartPositionAndQuat([0.5, -0.2, 0.6 - 0.1], [0, 1, 0, 0], duration=4)
    robot.gotoCartPositionAndQuat([0.5, -0.2, 0.52 - 0.1], [0, 1, 0, 0], duration=4)
