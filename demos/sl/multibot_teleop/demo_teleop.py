from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.sl.multibot_teleop.src.force_feedback_controller import (
    ForceFeedbackController,
)
from alr_sim.sims.sl.multibot_teleop.src.human_controller import HumanController
from alr_sim.sims.sl.multibot_teleop.src.teleop_controller import TeleopController
from alr_sim.sims.sl.multibot_teleop.src.ui.cli import ForceFeedbackCliMap

if __name__ == "__main__":
    sim_factory = SimRepository.get_factory("sl")

    scene = sim_factory.create_scene(skip_home=True)
    primary_robot = sim_factory.create_robot(
        scene,
        robot_name="panda2",
        backend_addr="tcp://141.3.53.152:51468",
        local_addr="141.3.53.158",
        gripper_actuation=True,
    )

    replica_robot = sim_factory.create_robot(
        scene,
        robot_name="panda1",
        backend_addr="tcp://141.3.53.151:51468",
        local_addr="141.3.53.158",
        gripper_actuation=True,
    )
    scene.start()

    replica_robot.gotoJointPosition(primary_robot.current_j_pos, block=True)

    primary_controller = ForceFeedbackController(primary_robot, replica_robot)
    replica_controller = TeleopController(
        lambda: primary_robot.current_j_pos,
        lambda: primary_robot.current_j_vel,
        lambda: primary_robot.gripper_width,
    )

    # cli
    cli = ForceFeedbackCliMap(primary_controller)
    cli.start()

    primary_controller.executeController(primary_robot, maxDuration=1000, block=False)
    replica_controller.executeController(replica_robot, maxDuration=1000)
