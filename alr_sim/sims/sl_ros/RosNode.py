"""
Define class RosNode
"""
import logging
import time
from typing import List

import rospy

from alr_sim.utils.gcd_lcm import lcm


class RosNode:
    def __init__(self, name, tick_step, components: List):
        assert name in self._valid_node_names
        self._realtime = self._check_real_time(name)

        try:
            self._node = rospy.init_node(
                name, anonymous=False, log_level=rospy.INFO, disable_signals=False
            )
        except rospy.exceptions.ROSException as e:
            logging.getLogger(__name__).info(
                "Node has already been initialized, do nothing"
            )

        self._components = list()
        self._MAIN_TICK_STEP = tick_step

        self._MAIN_TICK_MODULO = None
        self._main_tick = 0

        self.add_components(components)

    @property
    def _valid_node_names(self):
        return ["SimPanda", "Real_Panda1", "Real_Panda2", "Real_Panda3", "Real_Panda4"]

    @staticmethod
    def _check_real_time(name):
        if name == "SimPanda":
            realtime = False
        else:
            realtime = True
        return realtime

    def _update_main_tick_modulo(self):
        ticks = [component.tick for component in self._components]
        # The modulo should be the Least Common Multiple of the above ticks
        self._MAIN_TICK_MODULO = lcm(*ticks)

    def shutdown(self):
        pass

    def run(self):
        # rospy.is_shutdown()
        if not self._realtime:
            for component in self._components:
                if self._main_tick % component.tick == 0:
                    component.run_once()
            self._main_tick += self._MAIN_TICK_STEP
            self._main_tick %= self._MAIN_TICK_MODULO
        else:
            raise NotImplementedError

    def add_components(self, components):
        if components is not None:
            self._components += components
            self._update_main_tick_modulo()
