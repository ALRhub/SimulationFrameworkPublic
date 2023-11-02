import os
import shutil
import xml.etree.ElementTree as Et
from typing import List

from alr_sim.sims.mujoco import MujocoObject


class MjPhyCube(MujocoObject):
    """
    Cube Object for Mujoco Physics Tests.
    The cube has predefined properties and attached sensors.
    """

    SENSOR_DIMS = 12
    """Number of Sensor dimensions. Needed to compute Pointers of Mujoco Sensordata-Array"""

    CREATION_IDX = 0
    """Creation counter. Needed to compute Pointers of Mujoco Sensordata-Array"""

    def __init__(self, pos: List[float], quat: List[float]):
        super().__init__("", pos, quat)

        # Increase Creation Counter
        self.idx = MjPhyCube.CREATION_IDX
        MjPhyCube.CREATION_IDX += 1

        # Use Default template. Set flag if a modified copy of this template is used instead
        self.obj_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "assets", "MjPhyCube.xml")
        )
        self.copy = False

    @classmethod
    def reset(cls) -> None:
        """
        resets the creation counter
        Returns:
            None
        """
        cls.CREATION_IDX = 0

    def _copy_xml(self) -> None:
        """
        internal function. creates a copy of the template which can be modified safely.
        Returns:
            None
        """
        if self.copy:
            return

        copy_path = os.path.join(
            os.path.dirname(self.obj_path), "tmp_cube_{}.xml".format(self.idx)
        )
        shutil.copyfile(self.obj_path, copy_path)
        self.obj_path = copy_path
        self.copy = True

    def change_physics(
        self, solimp: List[float] = None, solref: List[float] = None
    ) -> None:
        """
        change the simulation physics properties of the cube by creating a modified copy of the template.
        See Mujoco Modeling Docu
        Args:
            solimp: solimp parameters
            solref: solref parameters

        Returns:
            None
        """
        if solimp is None and solref is None:
            return

        self._copy_xml()

        # Find Geom Tag
        tree = Et.parse(self.obj_path)
        wb = tree.find("worldbody")
        b = wb.find("body")
        geom = b.find("geom")

        # Modify Physic Attributes
        if solimp is not None:
            geom.set("solimp", " ".join(str(x) for x in solimp))
        if solref is not None:
            geom.set("solref", " ".join(str(x) for x in solref))

        # Save new XML file
        tree.write(self.obj_path)

    def parse_sensors(self, sensor_data: List[float]) -> dict:
        """
        parse the slice of the Mujoco Sensordata-Array.
        The XML template decides the order in which the sensor values are written to this array.
        Args:
            sensor_data: Mujoco Sensordata-Array slice

        Returns:
            dict of sensor-component name and corresponding value
        """
        return {
            "x": sensor_data[0],
            "y": sensor_data[1],
            "z": sensor_data[2],
            "xax1": sensor_data[3],
            "xax2": sensor_data[4],
            "xax3": sensor_data[5],
            "yax1": sensor_data[6],
            "yax2": sensor_data[7],
            "yax3": sensor_data[8],
            "zax1": sensor_data[9],
            "zax2": sensor_data[10],
            "zax3": sensor_data[11],
        }
