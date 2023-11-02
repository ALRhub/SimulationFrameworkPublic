import os

import test_demos.mj_physics.condim.CondimTestObject as cto
from alr_sim.sims.mujoco import MujocoScene
from test_demos.mj_physics.MjPhyRobot import MjPhyRobot
from alr_sim.utils import sim_path as sfp

if __name__ == "__main__":
    CONDIM = 6

    obj_list = [
        cto.CondimTestObject(
            type=cto.CondimTestObjectType.CYLINDER, pos=[-0.1, 1.5, 0.5], condim=CONDIM
        ),
        cto.CondimTestObject(
            type=cto.CondimTestObjectType.CYLINDER, pos=[0.3, 0.2, 0.1], condim=CONDIM
        ),
        cto.CondimTestObject(
            type=cto.CondimTestObjectType.CUBE, pos=[-0.1, 1.3, 0.5], condim=CONDIM
        ),
        cto.CondimTestObject(
            type=cto.CondimTestObjectType.CUBE, pos=[0.3, 0.0, 0.5], condim=CONDIM
        ),
    ]

    scene = MujocoScene(
        object_list=obj_list,
        panda_xml_path=sfp(
            "classic_framework/tests/mj_physics/condim/assets/CondimTestEnv.xml"
        ),
    )
    robot = MjPhyRobot(log_dir=os.path.join(os.path.dirname(__file__), "logs"))
    # robot.start_logging()

    robot.grasp([0.3, 0.0, 0.04])
    robot.gotoCartPositionAndQuat([0.3, 0.0, 0.15], [0, 1, 0, 0])
    robot.gotoCartPositionAndQuat([0.3, 0.0, 0.15], [0, 0, 1, 0])
    robot.grasp([0.3, 0.2, 0.04])
    robot.gotoCartPositionAndQuat([0.3, 0.2, 0.15], [0, 1, 0, 0])
    robot.gotoCartPositionAndQuat([0.3, 0.2, 0.15], [0, 0, 1, 0])
