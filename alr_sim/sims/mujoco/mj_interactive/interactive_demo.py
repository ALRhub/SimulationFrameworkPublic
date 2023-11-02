from alr_sim.sims.mujoco.mj_interactive.ia_ctrl import TCPController
from alr_sim.sims.mujoco.mj_interactive.ia_objects.pushing_object import (
    VirtualPushObject,
)
from alr_sim.sims.SimFactory import SimRepository

DEVICE = "COMBO"
DEVICE = "GAMEPAD"
# DEVICE = "PHYPHOX"

PHYPHOX_URL = "http://192.168.1.63:8080"
if __name__ == "__main__":
    c = None
    if DEVICE == "GAMEPAD":
        c = TCPController.TcpGamepadController()
    if DEVICE == "PHYPHOX":
        c = TCPController.TcpPhyPhoxController(PHYPHOX_URL)
    if DEVICE == "COMBO":
        c = TCPController.TcpComboController(PHYPHOX_URL)

    # Generate KitBash Objects
    obj_list = [
        VirtualPushObject(
            init_pos=[0.4, -0.2, 0.15], init_quat=[0, 1, 0, 0], rgba=[0, 0, 1, 1]
        )
    ]

    simulator = SimRepository.get_factory("mj_interactive")

    scene = simulator.create_scene(object_list=obj_list)
    robot = simulator.create_robot(scene, c)
    scene.start()
    robot.run()
