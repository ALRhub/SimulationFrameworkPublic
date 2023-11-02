import os
import xml.etree.ElementTree as Et

DEFAULT_XML_OUT = os.path.join(os.path.dirname(__file__), "assets", "mjphy_scene.xml")


class MjPhyParser:
    """
    Mujoco Scene Parser inspired XML manipulator to overwrite the physic settings for Mujoco Physics Tests
    """

    def __init__(self, output_path: str = DEFAULT_XML_OUT):
        self.output_path = output_path

    def change_physics(
        self, scene_xml_path: str, solimp: list = None, solref: list = None
    ) -> str:
        """
        creates a copy of the existing scene XML with overwritten physics parameters
        Args:
            scene_xml_path: path to original scene XML
            solimp: solimp paramters
            solref: solref parameters

        Returns:
            string path to the new XML file
        """
        tree = Et.parse(
            os.path.join(
                os.path.dirname(__file__),
                "../..",
                "..",
                "..",
                *scene_xml_path.split("/")
            )
        )
        options = tree.find("option")

        if solref is not None:
            options.set("o_solref", " ".join(str(x) for x in solref))

        if solimp is not None:
            options.set("o_solimp", " ".join(str(x) for x in solimp))

        flag = tree.find("flag")
        if flag is None:
            flag = Et.SubElement(options, "flag")

        flag.set("override", "enable")
        tree.write(self.output_path)
        return self.output_path
