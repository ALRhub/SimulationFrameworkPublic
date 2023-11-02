from enum import Enum
from typing import List

import numpy as np

from alr_sim.sims.mujoco import MujocoRobot
from test_demos.mj_physics.MjPhyLogging import MjPhyLogger


class RobotAction(Enum):
    # FLIP = 'flip'
    PUSH = "push"
    PRESS = "press"
    GRASP = "grasp"


class MjPhyRobot(MujocoRobot):
    """
    Robot for Mujoco Physics Tests.
    Includes pre-defined motion primitives and a Sensor Logger
    """

    def __init__(self, scene, log_dir=None, *args, **kwargs):
        super(MjPhyRobot, self).__init__(scene, *args, **kwargs)

        # Use MjPhyLogger for Sensor readings
        self.logger = MjPhyLogger(scene, log_dir)

    def do(self, action: RobotAction, target_pos=None) -> None:
        """
        perform a predefined motion primitive
        Args:
            action: type of action
            target_pos: target coordinate for the action

        Returns:
            None
        """
        # if action == RobotAction.FLIP:
        #    return self.flip_table()

        if target_pos is None:
            target_pos = [0.5, 0.0, 0.5]

        if action == RobotAction.PRESS:
            return self.press(target_pos)

        if action == RobotAction.PUSH:
            return self.push(target_pos)

        if action == RobotAction.GRASP:
            return self.grasp(target_pos)

        return self.press((target_pos))

    def press(self, target_pos: List[float]) -> None:
        """
        perform a vertical press
        Args:
            target_pos: target coordinates

        Returns:
            None
        """
        above = np.add(target_pos, [0, 0, 0.15])
        below = np.subtract(target_pos, [0, 0, 0.03])

        self._move_along(above, below)

    def push(self, target_pos) -> None:
        """
        perform a forward push motion along X-Axis
        Args:
            target_pos: target coordinate

        Returns:
            None
        """
        target_pos = np.subtract(target_pos, [0, 0, 0.05])
        front = np.subtract(target_pos, [0.1, 0, 0])
        back = np.add(target_pos, [0.1, 0, 0])

        self._move_along(front, back)

    def grasp(self, target_pos) -> None:
        self.set_gripper_width = 0.05

        above = np.add(target_pos, [0, 0, 0.15])
        below = np.subtract(target_pos, [0, 0, 0.04])

        self._move_along(above, below, False)
        self.set_gripper_width = 0.0
        self._move_along(below, above, False)

    def flip_table(self) -> None:
        """
        perform a motion which results in a flipped table
        Returns:
            None
        """
        self._move_along([0.0, 0.0, 0.1], [0.6, 0.0, 0.4])

    def _move_along(
        self, first_pos: List[float], second_pos: List[float], return_home: bool = True
    ) -> None:
        """
        internal function to perform a motion primitive.
        Each motion is defined via a start and end coordinate
        Args:
            first_pos: start coordinate
            second_pos: end coordinate

        Returns:
            None
        """
        quat = [0, 1, 0, 0]

        self.gotoCartPositionAndQuat(first_pos, quat)
        self.gotoCartPositionAndQuat(second_pos, quat)

        if return_home:
            self.gotoCartPositionAndQuat(first_pos, quat, duration=0.01)

            ## Go Home
            self.gotoCartPositionAndQuat([0.2, 0.0, 0.8], quat)
