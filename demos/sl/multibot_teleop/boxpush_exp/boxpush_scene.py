import numpy as np

from alr_sim.sims.mujoco.mj_interactive.ia_objects.pushing_object import (
    VirtualPushObject,
)
from alr_sim.sims.mujoco.mj_interactive.ia_robots.mj_push_robot import MjPushRobot
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.universal_sim.PrimitiveObjects import Sphere
from alr_sim.utils.geometric_transformation import euler2quat, quat2euler


def virt_scene(seed=42, render=True):
    """Create a virtual Scene"""
    np.random.seed(seed)

    # Bound Indicators
    obj_list = [
        Sphere(
            None,
            [0.65, 0.0, 0.0],
            [0, 1, 0, 0],
            visual_only=True,
            rgba=[1, 0, 0, 1],
            static=True,
        ),
        Sphere(
            None,
            [0.5, -0.35, 0.0],
            [0, 1, 0, 0],
            visual_only=True,
            rgba=[0, 1, 0, 1],
            static=True,
        ),
        Sphere(
            None,
            [0.30, -0.35, 0.0],
            [0, 1, 0, 0],
            visual_only=True,
            rgba=[0, 1, 0, 1],
            static=True,
        ),
        Sphere(
            None,
            [0.30, 0.25, 0.0],
            [0, 1, 0, 0],
            visual_only=True,
            rgba=[0, 1, 0, 1],
            static=True,
        ),
        Sphere(
            None,
            [0.5, 0.25, 0.0],
            [0, 1, 0, 0],
            visual_only=True,
            rgba=[0, 1, 0, 1],
            static=True,
        ),
    ]

    degrees = np.random.random_sample() * 180 - 90
    goal_angle = [0, 0, degrees * np.pi / 180]
    quat = euler2quat(goal_angle)

    x_pos = np.random.random_sample() * 0.2 + 0.3
    y_pos = np.random.random_sample() * 0.5 - 0.35
    obj_list.append(
        VirtualPushObject(
            [x_pos, y_pos, 0.0],
            quat,
            scale=1.3,
            rgba=[0, 1, 0, 0.3],
            visual_only=True,
            name="target_indicator",
        )
    )

    obj_list.append(
        VirtualPushObject(
            [0.65, 0.0, 0.016],
            [1, 0, 0, 0],
            rgba=[1, 0, 0, 1],
            scale=1.3,
            name="push_box",
        )
    )

    factory = SimRepository.get_factory("mujoco")

    obj_list.append(
        factory.create_camera(
            "boxpush_cam",
            init_pos=[1.2, 0.0, 1.2],
            init_quat=euler2quat([0, 0.7, np.pi / 2.0]),
        )
    )

    rm = factory.RenderMode.BLIND
    if render:
        rm = factory.RenderMode.HUMAN

    s = factory.create_scene(object_list=obj_list, render=rm)

    virtual_bot = MjPushRobot(scene=s)

    return s, virtual_bot


if __name__ == "__main__":
    scene, robot = virt_scene()
    scene.start()

    robot.wait(20)
    robot.wait(1)
