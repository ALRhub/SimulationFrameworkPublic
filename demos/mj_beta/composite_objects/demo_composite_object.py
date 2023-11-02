from alr_sim.core.logger import RobotPlotFlags
from alr_sim.sims.SimFactory import SimRepository
from alr_sim.sims.mj_beta.mj_utils.mj_scene_object import CompositeMujocoObject
import os

if __name__ == "__main__":
    dir_path = os.path.dirname(os.path.realpath(__file__))
    banana_path = os.path.join(dir_path, "011_banana")
    banana = CompositeMujocoObject(
        object_folder=banana_path,
        object_name="banana",
        pos=[0.4, 0, 0.05],
        quat=[0, 0, 0, 1],
    )

    object_list = [banana]

    # Setup the scene
    sim_factory = SimRepository.get_factory("mj_beta")

    scene = sim_factory.create_scene(object_list=object_list)
    robot = sim_factory.create_robot(scene)
    scene.start()

    robot.set_desired_gripper_width(0.4)  # we set the gripper to clos at the beginning

    # execute the pick and place movements
    robot.gotoCartPositionAndQuat(
        [0.4, 0, 0.1], [0, 0.7071068, -0.7071068, 0], duration=8
    )
    robot.gotoCartPositionAndQuat(
        [0.4, 0, -0.01], [0, 0.7071068, -0.7071068, 0], duration=2
    )
    robot.close_fingers()
    robot.gotoCartPositionAndQuat(
        [0.4, 0, 0.1], [0, 0.7071068, -0.7071068, 0], duration=8
    )
    robot.gotoCartPositionAndQuat(
        [0.8, 0, 0.1], [0, 0.7071068, -0.7071068, 0], duration=4
    )

    robot.open_fingers()

    robot.wait(10)

    scene.stop_logging()
