import enum
from xml.etree import ElementTree as Et

from alr_sim.sims.mujoco.MujocoLoadable import MujocoTemplateObject
from alr_sim.utils import sim_path as sfp


class CondimTestObjectType(enum.Enum):
    CUBE = sfp(
        "classic_framework",
        "tests",
        "mj_physics",
        "condim",
        "assets",
        "CondimTestCube.xml",
    )
    CYLINDER = sfp(
        "classic_framework",
        "tests",
        "mj_physics",
        "condim",
        "assets",
        "CondimTestSphere.xml",
    )


class CondimTestObject(MujocoTemplateObject):
    OBJ_COUNTER = 0

    def __init__(
        self,
        name: str = None,
        obj_type=CondimTestObjectType.CUBE,
        pos=None,
        size=None,
        condim=1,
    ):

        self.name = name

        if self.name is None:
            self.name = "condim_obj:{}".format(CondimTestObject.OBJ_COUNTER)
            CondimTestObject.OBJ_COUNTER += 1

        if pos is None:
            pos = [0, 0, 0]

        self.pos = pos
        self.size = size
        self.type = obj_type
        self.condim = condim

    def fill_template(self, body: Et.Element) -> Et.Element:
        body.set("name", self.name)
        body.set("pos", " ".join(map(str, self.pos)))

        geom = body.find("geom")
        if self.size is not None:
            geom.set("size", " ".join(map(str, self.size)))
        geom.set("condim", str(self.condim))
        return body

    @property
    def xml_file_path(self):
        return self.type.value
