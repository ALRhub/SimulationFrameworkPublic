import csv
import os

import matplotlib.pyplot as plt
import numpy as np

from alr_sim.core.logger import LoggerBase
from test_demos.mj_physics.sol_params.MjPhyScene import MjPhyScene


class MjPhyLogger(LoggerBase):
    """
    Logger for Sensordata from Mujoco Physics Tests.
    """

    def __init__(self, scene: MjPhyScene, log_dir=None):
        self.scene = scene
        self.data_array = []

        # Create directory for plots

        if log_dir is None:
            log_dir = os.path.join(os.path.dirname(__file__), "logs")
        self.out_dir = log_dir
        self.plot_dir = os.path.join(self.out_dir, "plots")
        self.isLogging = False
        os.makedirs(self.plot_dir, exist_ok=True)

    def _start(self):
        self.data_array = []

    def _log(self):
        data = self.scene.read_all_sensors()
        self.data_array.append(data)

    def _stop(self):
        with open(os.path.join(self.out_dir, "log.csv"), "w") as f:
            writer = csv.DictWriter(f, self.data_array[0].keys())
            writer.writeheader()
            writer.writerows(self.data_array)

    def plot(self, solimp=None, solref=None, action=None):
        # Cast Datastructure to Numpy
        array = np.asarray([list(d.values()) for d in self.data_array])

        # Create two stacked plots
        fig, (ax, ax2) = plt.subplots(2)

        # Plot Position Data
        ax.plot(array[:, 0] - 0.6, label="X-Pos")  # , color='tab:red')
        ax.plot(array[:, 1] - 0.0, label="Y-Pos")  # , color='tab:green')
        ax.plot(array[:, 2] - 0.02, label="Z-Pos")  # , color='tab:blue')

        # Convert Axes logs to angles in relation to world axes
        x_angle = np.arccos(np.dot(array[:, 3:6], np.asarray([1.0, 0.0, 0.0])))
        y_angle = np.arccos(np.dot(array[:, 6:9], np.asarray([0.0, 1.0, 0.0])))
        z_angle = np.arccos(np.dot(array[:, 9:12], np.asarray([0.0, 0.0, 1.0])))

        # Plot Axes angles
        ax2.plot(x_angle, label="X-Angle")  # , color='tab:red')
        ax2.plot(y_angle, label="Y-Angle")  # , color='tab:green')
        ax2.plot(z_angle, label="Z-Angle")  # , color='tab:blue')

        ax.legend(loc="best")
        ax2.legend(loc="best")

        fname = self._shorten_fname(solimp, solref, action)
        ax.set_title(fname)

        plt.savefig(os.path.join(self.plot_dir, fname.replace(".", "-") + ".svg"))
        plt.show()

    def _shorten_fname(self, solimp=None, solref=None, action="") -> str:
        """
        creates an unambiguous name depending on the simulation parameters
        Args:
            solimp: solimp parameter
            solref: solref parameter
            action: action name

        Returns:
            name string
        """
        if solimp is None and solref is None:
            return "default_" + action

        fname = ""

        if solimp is not None:
            fname += "solimp_"
            fname += "_{}_{}_".format(*solimp)

        if solref is not None:
            fname += "solref_"
            fname += "_{:.3f}_{:.2f}_".format(*solref)

        return fname + action
