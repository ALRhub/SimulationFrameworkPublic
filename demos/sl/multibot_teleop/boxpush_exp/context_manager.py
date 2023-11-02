import numpy as np

from alr_sim.utils.geometric_transformation import euler2quat


class BoxpushContextManager:
    def __init__(self, scene, seed=42) -> None:
        self.scene = scene

        np.random.seed(seed)

        self.deg_list = np.random.random_sample(60) * 180 - 90
        self.x_list = np.random.random_sample(60) * 0.2 + 0.3
        self.y_list = np.random.random_sample(60) * 0.5 - 0.35

        self.index = 0

    def start(self):
        self.set_context(self.index)

    def set_context(self, index):
        goal_angle = [0, 0, self.deg_list[index] * np.pi / 180]
        quat = euler2quat(goal_angle)
        self.scene.set_obj_pos_and_quat(
            [self.x_list[index], self.y_list[index], 0.00],
            quat,
            obj_name="target_indicator",
        )
        print("Set Context {}".format(index))

    def next_context(self):
        self.index = (self.index + 1) % len(self.deg_list)
        self.set_context(self.index)
