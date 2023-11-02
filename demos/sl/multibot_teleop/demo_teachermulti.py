import sys

from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.sl.multibot_teleop.src.human_controller import HumanController
from alr_sim.sims.sl.multibot_teleop.src.ui.gain_adjuster import GainAdjuster


def exit_callback(gain_adjuster):
    if gain_adjuster.stop:
        sys.exit()


if __name__ == "__main__":
    sim_factory = SimRepository.get_factory("sl")

    scene = sim_factory.create_scene(skip_home=True)
    left_robot = sim_factory.create_robot(
        scene,
        robot_name="panda2",
        backend_addr="tcp://141.3.53.152:51468",
        local_addr="141.3.53.158",
        gripper_actuation=True,
    )

    right_robot = sim_factory.create_robot(
        scene,
        robot_name="panda1",
        backend_addr="tcp://141.3.53.151:51468",
        local_addr="141.3.53.158",
        gripper_actuation=True,
    )

    scene.start()

    human_controller = HumanController(left_robot)
    human_controller = HumanController(right_robot)

    human_controller.executeController(left_robot, maxDuration=1000, block=False)
    human_controller.executeController(right_robot, maxDuration=1000)
