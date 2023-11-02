import os
import sys

from boxpush_exp.boxpush_logger import VTwinLogger
from boxpush_exp.boxpush_scene import virt_scene
from boxpush_exp.context_manager import BoxpushContextManager

from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.sl.multibot_teleop.src.human_controller import HumanController
from alr_sim.sims.sl.multibot_teleop.src.teleop_controller import (
    CartesianTeleopController,
    TeleopController,
)
from alr_sim.sims.sl.multibot_teleop.src.ui.cli import VTwinCLI


def main(index=0):
    # Real Robot
    sim_factory = SimRepository.get_factory("sl")
    scene = sim_factory.create_scene(skip_home=True)
    primary_robot = sim_factory.create_robot(
        scene,
        robot_name="panda2",
        backend_addr="tcp://192.168.122.185:51468",
        local_addr="192.168.122.1",
        # backend_addr="tcp://141.3.53.152:51468",
        # local_addr="141.3.53.158",
        gripper_actuation=True,
    )

    # Virtual Robot
    mj_scene, replica_robot = virt_scene()

    # Start Scenes and Sync Positions
    scene.start()
    mj_scene.start()
    replica_robot.beam_to_joint_pos(primary_robot.current_j_pos)

    manager = BoxpushContextManager(mj_scene)
    manager.index = index
    manager.start()

    # Create Teleop Controllers
    primary_controller = HumanController(primary_robot, regularize=True)
    # replica_controller = TeleopController(
    #     lambda: primary_robot.current_j_pos.copy(),
    #     lambda: primary_robot.current_j_vel.copy(),
    #     lambda: primary_robot.gripper_width,
    # )

    replica_controller = CartesianTeleopController(
        lambda: primary_robot.current_c_pos.copy(),
    )

    # Logger
    logger = VTwinLogger(os.path.join(os.path.dirname(__file__), "logs"), manager)

    # cli
    cli = VTwinCLI(
        primary_robot,
        primary_controller,
        replica_robot,
        replica_controller,
    )

    # Debug Command for testing movement
    def cmd_goto():
        # Stop Human Controller
        primary_robot.active_controller = None
        primary_robot.gotoCartPositionAndQuat([0.33, -0.3, 0.17], [0, 1, 0, 0])
        # Wait for one step. Maybe Race Condition?
        primary_robot.executeJointPosCtrlTimeStep(primary_robot.current_j_pos, 1)
        primary_robot.gotoCartPositionAndQuat([0.33, 0.3, 0.17], [0, 1, 0, 0])
        # Continue Human Controller Mode
        primary_controller.executeController(
            primary_robot, maxDuration=1000, block=False
        )

    # cli.register_function("G", "goto", cmd_goto)
    cli.register_function(
        "L", "log", lambda: logger.start_log(primary_robot, replica_robot)
    )
    cli.register_function(
        "A", "abort", lambda: logger.abort_log(primary_robot, replica_robot)
    )
    cli.register_function(
        "P", "save", lambda: logger.stop_log(primary_robot, replica_robot)
    )
    cli.register_function("J", "joints", lambda: print(primary_robot.current_j_pos))
    cli.register_function("N", "next", lambda: manager.next_context())

    # Start CLI Thread
    cli.start()

    # Start Teleop Controllers
    primary_controller.executeController(primary_robot, maxDuration=1000, block=False)
    replica_controller.executeController(replica_robot, maxDuration=1000, block=False)

    # Run Loop
    while not replica_controller.isFinished(replica_robot):
        scene.next_step()
        mj_scene.next_step()

    # Shutdown Procedure
    print("Goodbye")


if __name__ == "__main__":
    index = 0
    if len(sys.argv) > 1:
        index = int(sys.argv[1])

    main(index)
