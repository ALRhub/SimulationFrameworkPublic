import os

from test_demos.mj_physics.MjPhyRobot import MjPhyRobot, RobotAction
from test_demos.mj_physics.sol_params.MjPhyParams import MjPhyParams
from test_demos.mj_physics.sol_params.MjPhyScene import MjPhySceneFactory

RENDER = False


def run_test(solimp=None, solref=None, action: RobotAction = RobotAction.PRESS):
    with MjPhySceneFactory(render=RENDER) as factory:
        factory.change_cube_physics(solimp=solimp, solref=solref)

        scene = factory.create_scene()
        robot = MjPhyRobot(
            scene, log_dir=os.path.join(os.path.dirname(__file__), "logs")
        )
        scene.start()
        scene.start_logging()

        robot.do(action, factory.box_pos)

        scene.stop_logging()
        robot.logger.plot(solimp, solref, action=action.value)


if __name__ == "__main__":
    param_grid = MjPhyParams.get(False, True)

    total_runs = len(param_grid)

    for action in RobotAction:
        i = 1
        print(action)

        for solimp, solref in param_grid:
            # solref = [0.002, 0.5]
            print(solimp, solref)
            print("*** {}/{} ***".format(i, total_runs))
            print()
            run_test(solimp=solimp, solref=solref, action=action)
            i += 1
