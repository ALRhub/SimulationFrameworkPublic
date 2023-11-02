import os

from alr_sim.sims.mujoco import MujocoPrimitiveObject
from alr_sim.sims.mujoco.MujocoScene import MujocoScene
from test_demos.mj_physics.sol_params.MjPhyCube import MjPhyCube
from test_demos.mj_physics.sol_params.MjPhyParser import MjPhyParser


class MjPhyScene(MujocoScene):
    """
    Scene Object for Mujoco Physics Tests.
    Includes sensor readout functions.
    """

    def read_cube_sensors(self, cube: MjPhyCube) -> dict:
        """
        reads the sensor values of a single MjPhyCube
        Args:
            cube: cube object

        Returns:
            dict of sensor names and values
        """
        sensordata = self.sim.data.sensordata

        cube_data = sensordata[
            cube.idx * cube.SENSOR_DIMS : cube.idx * cube.SENSOR_DIMS + cube.SENSOR_DIMS
        ]
        return cube.parse_sensors(cube_data)

    def read_all_sensors(self):
        """
        reads the sensors of all objects in the scene.
        Returns:

        """
        for obj in self.object_list:
            if isinstance(obj, MjPhyCube):
                return self.read_cube_sensors(obj)


class MjPhySceneFactory:
    """
    Class to create a standardized Scene for Mujoco Physics Tests.
    """

    def __init__(
        self, render: bool = True, massive_table: bool = True, static_table: bool = True
    ):
        self.render = render

        # Use Default Scene, but prepare custom Parser
        self.scene_xml = None
        self.phy_parser = MjPhyParser()

        # Use standardized Setup
        self.box_pos = [0.6, 0.0, 0.05]
        self.object_list = []
        self.camera_list = None
        self._basic_setup(massive_table, static_table)

        # reset all physics to default values for safety
        self.reset_physics()

    def __enter__(self):
        """
        needed to make MjPhySceneFactory context
        Returns:

        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        reset all changed values when exiting context
        Args:
            exc_type:
            exc_val:
            exc_tb:

        Returns:

        """
        MjPhyCube.reset()

    def reset_physics(self) -> None:
        """
        reset the physics of the scene by loading a clean scene XML file
        Returns:
            None
        """
        self.scene_xml = "/envs/mujoco/panda/panda_mocap_model.xml"

    def change_global_physics(self, solimp: list = None, solref: list = None) -> None:
        """
        change the physic paramters of all the objects in the scene
        Args:
            solimp: solimp paramters
            solref: solref parameters

        Returns:
            None
        """
        self.scene_xml = self.phy_parser.change_physics(self.scene_xml, solimp, solref)

    def change_cube_physics(self, solimp: list = None, solref: list = None) -> None:
        """
        change the physics paramters for all the cubes in the scene
        Args:
            solimp: solimp paramters
            solref: solref parameters

        Returns:
            None
        """
        for obj in self.object_list:
            if isinstance(obj, MjPhyCube):
                obj.change_physics(solimp=solimp, solref=solref)

    def _basic_setup(
        self, massive_table: bool = True, static_table: bool = True
    ) -> None:
        """
        create a standardized scene setup consisting of a table and an MjPhyCube
        Args:
            massive_table: bool if a massive or thin table shall be used
            static_table: bool if the table shall be static or movable.

        Returns:
            None
        """
        table_pos = [0.5, 0.0, 0.2]
        table_size = [0.25, 0.35, 0.2]

        if not massive_table:
            table_pos[2] = 0.399
            table_size[2] = 0.001

        table = MujocoPrimitiveObject(
            obj_pos=table_pos,
            obj_name="table0",
            geom_size=table_size,
            mass=2000,
            geom_material="table_mat",
            static=static_table,
        )

        # self.add_object(table)

        b = MjPhyCube(self.box_pos, [1, 0, 0, 0])
        self.add_object(b)

    def add_object(self, obj) -> None:
        """
        Add an object to the scene
        Args:
            obj: object

        Returns:
            None
        """
        self.object_list.append(obj)

    def create_scene(self) -> MjPhyScene:
        """
        creates the resulting scene
        Returns:
            the created scene
        """
        scene_path = self.scene_xml

        if os.path.exists(scene_path):
            scene_path = "./" + os.path.relpath(
                scene_path, os.path.join(os.path.dirname(__file__), "../..", "..", "..")
            )

        return MjPhyScene(
            object_list=self.object_list,
            camera_list=self.camera_list,
            panda_xml_path="./classic_framework/tests/mj_physics/sol_params/assets/reach.xml",
            control="mocap",
            gripper_control="None",
            render=self.render,
        )
