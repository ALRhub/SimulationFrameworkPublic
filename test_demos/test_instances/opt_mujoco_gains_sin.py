"""
Using sin trajectory to optimize gains
"""

from test_demos.traj_tests.optimize_gains import OptimizeMujocoGains
from pathlib import Path
from test_demos.traj_tests.follow_traj_test import mujoco_joint_gain_test

# Load optimization and objective function config files
module_path = str(Path(__file__).resolve().parents[0])
opt_config_path = "../traj_tests/test_config/mujoco_params/PD_control_gains_test"
obj_config_path = "../traj_tests/test_config/trajs/sin_traj_period4_mag05"
save_to_dir = "../test_results"

# Optimize without saving to json file
# omg = OptimizeMujocoGains("cma_es", opt_config_path, obj_config_path,
#                           save_to=None)

# Optimize and saving to file
omg = OptimizeMujocoGains(
    "cma_es", opt_config_path, obj_config_path, save_to=save_to_dir
)


best_params = omg.optimize()

# Load result and test
gains = list(best_params.values())

# Example code Loading result from json
#
# path = "..."
# with open(path) as fp:
#     gains = json.load(fp).values()

mujoco_joint_gain_test(obj_config_path, gains=gains, render=True, plotting=True)
