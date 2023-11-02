import abc

import numpy as np

from alr_sim.sims.mujoco import MujocoScene, MujocoRobot
from alr_sim.sims.sl_ros.InHandTransformer import InHandTransformer
from alr_sim.sims.universal_sim.PrimitiveObjects import Box
from alr_sim.utils import tcp_transform


class Tracker(Box, abc.ABC):
    ID = 0

    def __init__(self, rgba=None):
        if rgba is None:
            rgba = [0, 1, 0, 1]
        super(Tracker, self).__init__(
            "tcp_tracker" + str(Tracker.ID),
            [1, 1, 1],
            [0, 1, 0, 0],
            size=[0.01, 0.01, 0.01],
            rgba=rgba,
            visual_only=True,
            static=True,
        )
        Tracker.ID += 1

    def pb_load(self, pb_sim) -> int:
        # Mujoco Suppport only of self.sync()
        pass

    @abc.abstractmethod
    def corrected_pos_quat(self, robot):
        return robot.current_c_pos, robot.current_c_quat

    def sync(self, scene: MujocoScene, robot: MujocoRobot):
        # Compute TCP Offset
        pos, quat = self.corrected_pos_quat(robot)

        # Beam visual marker to new TCP Pos. Mujoco Specific Tricks
        body_id = scene.sim.model.body_name2id(self.name)
        scene.sim.model.body_pos[body_id] = pos
        scene.sim.model.body_quat[body_id] = quat


class TcpOffsetTracker(Tracker):
    def corrected_pos_quat(self, robot):
        return tcp_transform.corrected_pos_quat(robot)


class MatrixTracker(Tracker):
    def __init__(self):
        super(MatrixTracker, self).__init__([0, 0, 1, 1])
        mat = np.array(
            [
                [1, 0, 0, 0.1],
                [0, 0, 1, 0],
                [0, 0, 1, -0.1],
                [0, 0, 0, 1],
            ]
        )
        self._transformer = InHandTransformer(mat)

    def corrected_pos_quat(self, robot):
        return self._transformer.get_pos_quat(robot)
