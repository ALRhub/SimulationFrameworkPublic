import time

import numpy as np


from alr_sim.sims.SimFactory import SimRepository
from envs.nut_assembly_env.nut_assembly import NutAssemblyEnv

import envs.nut_assembly_env.objects.nuts as nuts
import envs.nut_assembly_env.objects.pegs as pegs

if __name__ == "__main__":
    # simulator = "pybullet"
    simulator = "mujoco"

    env = NutAssemblyEnv(simulator=simulator)
    env.start()

    for _ in range(10000):
        env.step(np.array([0, 0, 0, 0, 0, 0, 0]))
        time.sleep(1.0 / 240.0)
