from classic_framework.mujoco.MujocoRobot import MujocoRobot
from classic_framework.mujoco.MujocoScene import MujocoScene
import classic_framework.mujoco.mujoco_utils.mujoco_controllers as mj_ctrl
from demos.mujoco.kitbash.procedural_objects import KitBashContainer

if __name__ == "__main__":

    # Generate KitBash Objects
    obj_list = []
    for i in range(20):
        obj_list.append(KitBashContainer())

    scene = MujocoScene(object_list=obj_list,
                        # panda_xml_path=sim_framework_path('envs', 'mujoco', 'panda', 'panda_gym.xml'),
                        control=mj_ctrl.MocapControl(),
                        # gripper_control='none',
                        )
    robot = MujocoRobot(scene)

    for i in range(10000):
        robot.gotoCartPositionAndQuat([0.5, 0.0, 0.5], [0, 1, 0, 0])
