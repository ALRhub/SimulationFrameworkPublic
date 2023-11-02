import numpy as np

from alr_sim.core.logger import RobotPlotFlags, ObjectLogger
from alr_sim.sims.SimFactory import SimRepository

if __name__ == "__main__":

    # Setup the scene
    sim_factory = SimRepository.get_factory("sl")

    scene = sim_factory.create_scene(skip_home=False)
    robot = sim_factory.create_robot(
        scene,
        robot_name="panda2",
        backend_addr="tcp://141.3.53.152:51468",
        local_addr="141.3.53.26",
        gripper_actuation=True,
    )

    robot2 = sim_factory.create_robot(
        scene,
        robot_name="panda1",
        backend_addr="tcp://141.3.53.151:51468",
        local_addr="141.3.53.26",
        gripper_actuation=True,
    )

    scene.start()

    des_q = np.array(
        [
            -0.17372284,
            0.74377287,
            -0.15055875,
            -1.8271288,
            0.17003154,
            2.52458572,
            1.85687575,
        ]
    )
    duration = 5
    # robot.go_home()
    # robot2.go_home()
    robot.gotoJointPosition(des_q, duration=duration, block=False)
    robot2.gotoJointPosition(des_q, duration=duration)
