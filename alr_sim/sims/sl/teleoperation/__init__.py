import warnings

from .src import *

warnings.warn(
    "The alr_sim.sims.sl.teleoperation package is deprecated. Please use alr_sim.sims.sl.multibot_teleop instead",
    DeprecationWarning,
)
