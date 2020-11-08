import abc
import os
import xml.etree.ElementTree as Et
from typing import Tuple

import classic_framework.utils.sim_path as sim_path


class MujocoLoadable(abc.ABC):
    """
    Interface for all kind of objects which can be loaded to a Mujoco Scene.
    Each class is responsible themselves to return a valid XML element which will be added to the Scene XML.
    """

    @abc.abstractmethod
    def to_xml(self, scene_dir: str) -> Tuple[Et.Element, bool]:
        """
        Function to convert the object to a valid XML node. Needs to be implemented by the subclass.
        Args:
            scene_dir: Directory of the scene XML. Used to built relative paths for <include /> nodes

        Returns:
            XML Element, bool if it is an include element.
        """
        raise NotImplementedError


class MujocoXmlLoadable(MujocoLoadable):
    """
    Interface to include a static XML file.
    """
    @property
    @abc.abstractmethod
    def xml_file_path(self):
        """
        Returns:
           path to the static xml file.
        """
        pass

    def to_xml(self, scene_dir: str) -> Tuple[Et.Element, bool]:
        include = Et.Element('include')
        include.set('file', os.path.relpath(self.xml_file_path, scene_dir))
        return include, True


class MujocoPrimitiveObject(MujocoLoadable):
    """
    Class representing an object for a MuJoCo sim environment. In order to load the object to the scene, it has to be
    given to the :class:`Scene` constructor.
    """

    def __init__(self,
                 obj_name,
                 obj_pos,
                 mass=0.1,
                 inertial_pos=None,
                 diaginertia=None,
                 geom_name=None,
                 geom_type="box",
                 geom_pos=None,
                 geom_material=None,
                 geom_size=None,
                 geom_rgba=None,
                 static=False) -> None:
        self.obj_name = obj_name
        self.obj_pos = obj_pos
        self.mass = mass

        if inertial_pos is None:
            inertial_pos = [0, 0, 0]
        else:
            assert len(geom_size) == 3, "Error, parameter inertial_pos has to be three dimensional."
        self.inertial_pos = inertial_pos

        if diaginertia is None:
            diaginertia = [1, 1, 1]  # for a cube...
        else:
            assert len(diaginertia) == 3, "Error, parameter diaginertia has to be three dimensional."
        self.diaginertia = diaginertia

        if geom_name is None:
            geom_name = obj_name + ":geom1"
        self.geom_name = geom_name
        self.geom_type = geom_type

        if geom_pos is None:
            geom_pos = [0, 0, 0]
        else:
            assert len(geom_size) == 3, "Error, parameter geom_pos has to be three dimensional."
        self.geom_pos = geom_pos
        self.geom_material = geom_material

        if geom_size is None:
            geom_size = [0.02, 0.02, 0.02]
        else:
            assert len(geom_size) == 3, "Error, parameter geom_size has to be three dimensional."
        self.geom_size = geom_size

        if geom_rgba is not None:
            assert len(geom_rgba) == 4, "Error, parameter geom_rgba has to be four dimensional."
        self.geom_rgba = geom_rgba
        self.static = static

    def to_xml(self, scene_dir: str) -> Tuple[Et.Element, bool]:
        # cast types to string for xml parsing
        obj_pos_str = ' '.join(map(str, self.obj_pos))
        inertial_pos_str = ' '.join(map(str, self.inertial_pos))
        mas_str = str(self.mass)
        diaginertia_str = ' '.join(map(str, self.diaginertia))
        geom_pos_str = ' '.join(map(str, self.geom_pos))
        geom_size_str = ' '.join(map(str, self.geom_size))

        # Create new object for xml tree
        object_body = Et.Element('body')
        object_body.set('name', self.obj_name)
        object_body.set('pos', obj_pos_str)

        # Set object parameters
        if not self.static:
            freejoint = Et.SubElement(object_body, 'freejoint')

        inertial = Et.SubElement(object_body, 'inertial')
        inertial.set('pos', inertial_pos_str)
        inertial.set('mass', mas_str)
        inertial.set('diaginertia', diaginertia_str)

        geom = Et.SubElement(object_body, 'geom')
        geom.set('name', self.geom_name)
        geom.set('type', self.geom_type)
        geom.set('pos', geom_pos_str)
        if self.geom_material is not None:
            geom.set('material', self.geom_material)
        geom.set('size', geom_size_str)

        if self.geom_rgba is not None:
            geom_rgba_str = ' '.join(map(str, self.geom_rgba))
            geom.set('rgba', geom_rgba_str)

        return object_body, False


class MujocoObject(MujocoLoadable):
    def __init__(self, object_name, pos, quat, root=sim_path.FRAMEWORK_DIR):

        if pos is None:
            pos = [0, 0, 0]
        else:
            assert len(pos) == 3, "Error, parameter pos has to be three dimensional."

        if quat is None:
            quat = [0, 0, 0, 0]
        else:
            assert len(quat) == 4, "Error, parameter quat has to be three dimensional."

        self.obj_path = os.path.join(root, './simulation/envs/mujoco/assets/' + object_name + '.xml')
        self.pos = pos
        self.quat = quat

    def to_xml(self, scene_dir: str) -> Tuple[Et.Element, bool]:
        obj = Et.parse(self.obj_path)
        worldbody = obj.find('worldbody')
        body = worldbody.find('body')

        # cast types to string for xml parsing
        obj_pos_str = ' '.join(map(str, self.pos))
        obj_quat_str = ' '.join(map(str, self.quat))

        body.set('pos', obj_pos_str)
        body.set('quat', obj_quat_str)

        obj.write(self.obj_path)

        include = Et.Element('include')
        include.set('file', os.path.relpath(self.obj_path, scene_dir))
        return include, True


class MujocoCamera(MujocoLoadable):
    def __init__(self,
                 cam_name,
                 cam_pos,
                 cam_euler,
                 cam_mode="fixed",
                 fovy="45",
                 ipd=0.068, ):
        assert len(cam_pos) == 3, "Error, cam_pos has to be three dimensional"
        assert len(cam_euler) == 3, "Error, cam_euler has to be three dimensional"

        self.cam_name = cam_name
        self.cam_pos = cam_pos
        self.cam_euler = cam_euler
        self.cam_mode = cam_mode
        self.fovy = fovy
        self.ipd = ipd

    def to_xml(self, scene_dir: str) -> Tuple[Et.Element, bool]:
        # cast types to string for xml parsing
        cam_pos_str = ' '.join(map(str, self.cam_pos))
        cam_euler_str = ' '.join(map(str, self.cam_euler))
        cam_fovy_str = str(self.cam_fovy)
        cam_ipd_str = str(self.cam_ipd)

        # Create new camera for xml tree
        cam = Et.Element('camera')
        cam.set('name', self.cam_name)
        cam.set('mode', self.cam_mode)
        cam.set('fovy', cam_fovy_str)
        cam.set('ipd', cam_ipd_str)
        cam.set('pos', cam_pos_str)
        cam.set('euler', cam_euler_str)

        return cam, False

class MujocoWorkspace(MujocoXmlLoadable):
    def __init__(self, size: str):
        if size not in ['small', 'medium', 'large']:
            raise ValueError(
                "Error, please choose a size between <small>, <medium> or <large> .")
        self.size = size

    @property
    def xml_file_path(self):
        return sim_path.sim_framework_path('./envs/mujoco/assets/workspace/{}_workspace.xml'.format(self.size))

