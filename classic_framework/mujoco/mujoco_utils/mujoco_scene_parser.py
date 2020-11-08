import logging
import os
import xml.etree.ElementTree as Et
from shutil import copyfile
from xml.etree.ElementTree import Element
from xml.etree.ElementTree import SubElement

from mujoco_py import load_model_from_path, MjSim

from classic_framework.mujoco.mujoco_utils.mujoco_controllers import MujocoController
from classic_framework.mujoco.mujoco_utils.mujoco_scene_object import MujocoLoadable
from classic_framework.utils.sim_path import sim_framework_path


class MujocoSceneParser:
    """
    Class for parsing assets to the xml file for setting up the scene for a mujoco environment.
    New created assets are written at the end of the worldbody element which specifies the assets in the environment.
    """

    def __init__(self, panda_xml_path='./envs/mujoco/panda/panda_with_cam_mujoco.xml'):

        src = sim_framework_path(panda_xml_path)
        self.scene_xml = sim_framework_path('./envs/mujoco/panda/main_scene.xml')
        copyfile(src, self.scene_xml)
        self._xml_path = sim_framework_path('./envs/mujoco/panda/')
        self._tree = Et.parse(sim_framework_path(self._xml_path, 'main_scene.xml'))
        self._root = self._tree.getroot()
        self._worldbody = self._root.find('worldbody')
        assert self._worldbody, 'Error, xml file does contain a world body.'

    def load_mj_loadable(self, mj_loadable: MujocoLoadable):
        xml_element, include = mj_loadable.to_xml(self._xml_path)
        if include:
            self._root.append(xml_element)
        else:
            self._worldbody.append(xml_element)  # append the new object to the worldbody of the xml file
        self.indent(self._root)  # ensures correct indendation
        self._tree.write(sim_framework_path(self._xml_path, 'main_scene.xml'))  # writes the changes to the file

    def set_control(self, control, set_gripper=True, gripper_ctrl=None, kv=None, kv_scale=1, kp=None, kp_scale=1):
        if isinstance(control, MujocoController):
            self.load_mj_loadable(control)
            if set_gripper:
                if gripper_ctrl is None:
                    gripper_ctrl = control.gripper_ctrl_name
                self.set_gripper_control(gripper_ctrl, kv, kv_scale, kp, kp_scale)
        else:
            self._set_control_by_name(control, set_gripper, gripper_ctrl, kv, kv_scale, kp, kp_scale)
            logging.warning(
                "Please use mujoco_controllers.MujocoControl objects. Setting a controller by name will be deprecated in the future.")

    def _set_control_by_name(self, control_name: str, set_gripper=True, gripper_ctrl=None, kv=None, kv_scale=1, kp=None,
                             kp_scale=1):
        # TODO: Remove in future!
        """
        Sets the control for the robot by writing the respective file to the xml.
        Args:
            control_name:
                Choose between:
                <ik> (inverse kinematics),
                <position> (forward kinematics),
                <torque> (torque based control) or
                <velocity> (joint velocity control).
            kv:
                Velocity feedback gain for each joint (list with num joints entries)
            kv_scale:
                Scales each Velocity feedback gain by a scalar (single value)
            kp:
                Position feedback gain for each joint (list with num joints entries)
            kp_scale:
                Scales each Position feedback gain by a scalar (single value)

        Returns:
            No return value.
        """
        include = Element('include')
        if control_name == 'ik' or control_name == 'torque':  # inverse kinematics / torque control
            include.set('file', './controller/panda_torque_control.xml')
            self._root.append(include)
            if gripper_ctrl is None:
                gripper_ctrl = 'ik'

        elif control_name == 'position':  # position control
            self.set_kp(kp=kp, scale=kp_scale)
            include.set('file', './controller/panda_position_control.xml')
            self._root.append(include)
            if gripper_ctrl is None:
                gripper_ctrl = 'ik'

        elif control_name == 'velocity':  # joint velocity control
            self.set_kv(kv=kv, scale=kv_scale)
            include.set('file', './controller/panda_velocity_control.xml')
            self._root.append(include)
            if gripper_ctrl is None:
                gripper_ctrl = 'ik'

        elif control_name == 'mocap':  # mocap control
            include.set('file', './controller/panda_mocap_control.xml')
            self._root.append(include)
            # self.add_mocap()
            if gripper_ctrl is None:
                gripper_ctrl = 'ik'

        else:
            raise ValueError("Error, invalid control value. Choose between <ik> (inverse kinematics), "
                             "<position> (forward kinematics), <torque> (torque based control), <velocity> (joint "
                             "velocity control) or <mocap> (mocap body based control).")
        if set_gripper:
            self.set_gripper_control(control=gripper_ctrl)

        self.indent(self._root)  # ensures correct indentation
        self._tree.write(sim_framework_path(self._xml_path, 'main_scene.xml'))  # writes the changes to the file

    def set_gripper_control(self, control, kv=None, kv_scale=20, kp=None, kp_scale=1):
        if control.lower() == 'none':
            return

        # Set the gripper control
        gripper_ctrl = Element('actuator')

        if control == 'ik' or control == 'torque' or control == 'mocap':  # inverse kinematics / torque control
            left_gripper = SubElement(gripper_ctrl, 'motor')
            # left_gripper.set('ctrlrange', "0 0.04")
            right_gripper = SubElement(gripper_ctrl, 'motor')
            # right_gripper.set('ctrlrange', "0 0.04")

        elif control == 'position':  # position control
            if kp is None:
                kp = str(kp_scale * 300)

            left_gripper = SubElement(gripper_ctrl, 'position')
            left_gripper.set('ctrlrange', "0 0.04")
            left_gripper.set('kp', kp)

            right_gripper = SubElement(gripper_ctrl, 'position')
            right_gripper.set('ctrlrange', "0 0.04")
            right_gripper.set('kp', kp)

        elif control == 'velocity':  # joint velocity control
            if kv is None:
                kv = str(kv_scale)

            left_gripper = SubElement(gripper_ctrl, 'velocity')
            left_gripper.set('ctrlrange', "-0.2 0.2")
            left_gripper.set('ctrllimited', "true")
            left_gripper.set('kv', kv)

            right_gripper = SubElement(gripper_ctrl, 'velocity')
            right_gripper.set('ctrlrange', "-0.2 0.2")
            right_gripper.set('ctrllimited', "true")
            right_gripper.set('kv', kv)

        else:
            raise ValueError("Error, invalid control value. Choose between <ik> (inverse kinematics), "
                             "<position> (forward kinematics), <torque> (torque based control), <velocity> (joint "
                             "velocity control) or <mocap> (mocap body based control).")

        left_gripper.set('joint', "panda_finger_joint1")
        left_gripper.set('name', "panda_finger_joint1_act")

        right_gripper.set('joint', "panda_finger_joint2")
        right_gripper.set('name', "panda_finger_joint2_act")

        right_gripper.set('forcerange', "-70 70")
        # right_gripper.set('forcelimited', "true")
        left_gripper.set('forcerange', "-70 70")
        # left_gripper.set('forcelimited', "true")

        self._root.append(gripper_ctrl)
        self.indent(self._root)  # ensures correct indentation
        self._tree.write(sim_framework_path(self._xml_path, 'main_scene.xml'))  # writes the changes to the file

    def set_kp(self, kp=None, scale=1):
        """
        Sets the position feedback gain for joint position control by writing the specified values to
        "panda_position_control.xml".

        Args:
            kp: Position feedback gain
            scale: Scales the Position feedback gain

        Returns:
            No return value
        """
        if kp is None:
            kp = [870, 870, 870, 870, 120, 120, 120]
        else:
            assert len(kp) == 7, ("Error, the number of entries in <kp> has to match the number of joints (i.e. 7). "
                                  "Given: ", len(kp))

        tree = Et.parse(sim_framework_path(self._xml_path, 'panda_position_control.xml'))
        root = tree.getroot()
        positions = root.find('actuator').findall('position')

        for i, pos in enumerate(positions):
            pos.set('kp', str(scale * kp[i]))

        tree.write(sim_framework_path(self._xml_path, 'panda_position_control.xml'))

    def set_kv(self, kv=None, scale=1):
        """
        Sets the velocity feedback gain for joint velocity control by writing the specified values to
        "panda_velocity_control.xml".

        Args:
            kv: Velocity feedback gain
            scale: Scales the Velocity feedback gain

        Returns:
            No return value
        """
        if kv is None:
            kv = [12, 12, 12, 12, 6, 5, 3]
        else:
            assert len(kv) == 7, ("Error, the number of entries in <kv> has to match the number of joints (i.e. 7). "
                                  "Given: ", len(kv))

        tree = Et.parse(sim_framework_path(self._xml_path, 'panda_velocity_control.xml'))
        root = tree.getroot()
        velocities = root.find('actuator').findall('velocity')

        for i, vel in enumerate(velocities):
            vel.set('kv', str(scale * kv[i]))

        tree.write(sim_framework_path(self._xml_path, 'panda_velocity_control.xml'))

    def add_mocap(self):
        """
        Creates a mocap body and adds it to the .xml creating the scene.

        Returns:
            No return value
        """
        # Create mocap body
        object_body = Element('body')
        object_body.set('mocap', 'true')
        object_body.set('name', 'panda:mocap')
        object_body.set('pos', '0 0 0')

        # Set the params of the mocap
        geom = SubElement(object_body, 'geom')
        geom.set('conaffinity', "0")
        geom.set('contype', "0")
        geom.set('pos', "0 0 0")
        geom.set('rgba', "0 0.5 0 0.7")
        geom.set('size', "0.005 0.005 0.005")
        geom.set('type', "box")

        geom1 = SubElement(object_body, 'geom')
        geom1.set('conaffinity', "0")
        geom1.set('contype', "0")
        geom1.set('pos', "0 0 0")
        geom1.set('rgba', "1 0 0 0.3")
        geom1.set('size', "1 0.005 0.005")
        geom1.set('type', "box")

        geom2 = SubElement(object_body, 'geom')
        geom2.set('conaffinity', "0")
        geom2.set('contype', "0")
        geom2.set('pos', "0 0 0")
        geom2.set('rgba', "0 1 0 0.3")
        geom2.set('size', "0.005 1 0.005")
        geom2.set('type', "box")

        geom3 = SubElement(object_body, 'geom')
        geom3.set('conaffinity', "0")
        geom3.set('contype', "0")
        geom3.set('pos', "0 0 0")
        geom3.set('rgba', "0 0 1 0.3")
        geom3.set('size', "0.005 0.005 1")
        geom3.set('type', "box")

        # Write the defined mocap to the worldbody of the scene
        self._worldbody.append(object_body)  # append the new object to the worldbody of the xml file
        self.indent(self._root)  # ensures correct indentation
        self._tree.write(sim_framework_path(self._xml_path, 'main_scene.xml'))  # writes the changes to the file

    def set_dt(self, dt=0.001):
        """
        Sets 1 / number of timesteps mujoco needs for executing one second of wall-clock time by writing to the root
        xml.file.

        Args:
            dt:
                1 / number of timesteps needed for computing one second of wall-clock time

        Returns:
            No return value

        """
        options = self._root.find('option')
        options.set('timestep', str(dt))
        self.indent(self._root)  # ensures correct indentation
        self._tree.write(sim_framework_path(self._xml_path, 'main_scene.xml'))  # writes the changes to the file

    def indent(self, elem: Element, level=0):
        """
        Ensures that the element which is written to the xml file is correctly indented.

        Returns:
            No return value
        """
        i = "\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                self.indent(elem, level + 1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    def create_sim(self, nsubsteps):
        model = load_model_from_path(self.scene_xml)
        sim = MjSim(model=model, nsubsteps=nsubsteps)
        os.remove(self.scene_xml)
        return sim, model
