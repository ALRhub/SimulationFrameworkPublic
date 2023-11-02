import numpy as np

from alr_sim.core.Robots import RobotBase
from alr_sim.utils import geometric_transformation as geom
from alr_sim.utils import tcp_transform


class InHandTransformer:
    """Workaround class, until tf2_ros implementation is running"""

    def __init__(self, transform_matrix: np.ndarray, attached_robot: RobotBase = None):
        if transform_matrix.shape != (4, 4):
            raise ValueError("Transform Matrix must be 4x4")

        self._matrix = transform_matrix
        self._robot = attached_robot

    def get_pos_quat(self, robot: RobotBase = None):
        if self._robot is not None:
            robot = self._robot

        if robot is None:
            raise ValueError("Missing Robot for Inhand Transform")

        return tcp_transform.corrected_pos_quat(
            robot, self._matrix[:3, 3], geom.mat2quat(self._matrix[:3, :3])
        )
