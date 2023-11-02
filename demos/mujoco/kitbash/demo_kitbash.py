import numpy as np

from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.mujoco.mj_interactive.ia_robots.mj_push_robot import MjPushRobot
from procedural_objects import KitBashContainer

if __name__ == "__main__":
    np.random.seed(0)

    # Generate KitBash Objects
    obj_list = []
    for i in range(20):
        obj_list.append(KitBashContainer())

    simulator = SimRepository.get_factory("mujoco")

    scene = simulator.create_scene(
        object_list=obj_list, render=simulator.RenderMode.HUMAN
    )
    robot = MjPushRobot(scene)
    scene.start()

    robot.set_desired_gripper_width(0.04)

    robot.gotoCartPositionAndQuat([0.5, 0.0, 0.5], [0, 1, 0, 0])

    robot.wait(duration=10)
