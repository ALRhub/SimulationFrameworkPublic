import abc

import numpy as np

from alr_sim.sims.mujoco.mj_interactive.devices.gamepad import GamePad
from alr_sim.sims.mujoco.mj_interactive.devices.phyphox import PhyPhoxIMU
from alr_sim.utils import geometric_transformation as geo_trans


class InteractiveTcpControllerBase(abc.ABC):
    def __init__(self, absolute_orientation=False):
        self.quat_operation = lambda x, y: geo_trans.quat_mul(x, y)
        if absolute_orientation:
            self.quat_operation = lambda x, y: x + y

    def move(self, tcp_pos, tcp_quat):
        new_pos = tcp_pos + self.read_ctrl_pos()
        new_quat = self.quat_operation(tcp_quat, self.read_ctrl_quat())
        return new_pos, new_quat

    def grip(self, gripper_width):
        new_target = gripper_width + self.read_grip_change() * 0.05

        return np.clip(new_target, 0.001, 0.1)

    @abc.abstractmethod
    def reset(self):
        pass

    @abc.abstractmethod
    def stop(self):
        pass

    @abc.abstractmethod
    def plot(self):
        pass

    @abc.abstractmethod
    def save(self):
        pass

    @abc.abstractmethod
    def read_ctrl_pos(self):
        pass

    @abc.abstractmethod
    def read_ctrl_quat(self):
        pass

    @abc.abstractmethod
    def read_grip_change(self):
        pass


class TcpGamepadController(InteractiveTcpControllerBase):
    def __init__(self, absolute_orientation=False):
        self.ctrl_device = GamePad()
        self.rot_dampening = 20.0
        self.pos_dampening = 50.0

        if absolute_orientation:
            self.rot_dampening = 1.0

        super(TcpGamepadController, self).__init__(absolute_orientation)

    def reset(self):
        return self.ctrl_device.BTN_THUMBL == 1

    def stop(self):
        return self.ctrl_device.BTN_START == 1

    def plot(self):
        return self.ctrl_device.BTN_SELECT == 1

    def save(self):
        return self.ctrl_device.BTN_NORTH == 1

    def read_ctrl_pos(self):
        return (
            np.array(
                [
                    self.ctrl_device.ABS_Y,
                    self.ctrl_device.ABS_X,
                    self.ctrl_device.BTN_TR - self.ctrl_device.BTN_TL,
                ]
            )
            / self.pos_dampening
        )

    def read_ctrl_quat(self):
        return geo_trans.euler2quat(
            np.array(
                [
                    self.ctrl_device.ABS_RX * np.pi,
                    self.ctrl_device.ABS_RY * np.pi,
                    (self.ctrl_device.ABS_RZ - self.ctrl_device.ABS_Z) * np.pi,
                ]
            )
            / self.rot_dampening
        )

    def read_grip_change(self):
        return -1 * self.ctrl_device.BTN_SOUTH + self.ctrl_device.BTN_EAST


class TcpPhyPhoxController(InteractiveTcpControllerBase):
    def __init__(self, url):
        self.ctrl_device = PhyPhoxIMU(url)
        super(TcpPhyPhoxController, self).__init__(True)

    def reset(self):
        return False

    def stop(self):
        return False

    def plot(self):
        return False

    def read_ctrl_pos(self):
        return (
            np.array(
                [self.ctrl_device.xPos, self.ctrl_device.yPos, self.ctrl_device.zPos]
            )
            / 1.0
        )

    def read_ctrl_quat(self):
        return geo_trans.euler2quat(
            np.array(
                [self.ctrl_device.xRot, self.ctrl_device.yRot, self.ctrl_device.zRot]
            )
            / 1.0
        )

    def read_grip_change(self):
        return 0


class TcpComboController(InteractiveTcpControllerBase):
    def __init__(self, url):
        self.rot_ctrl = TcpPhyPhoxController(url)
        self.pos_ctrl = TcpGamepadController()
        super(TcpComboController, self).__init__(True)

    def reset(self):
        return self.pos_ctrl.reset()

    def stop(self):
        return self.pos_ctrl.stop()

    def plot(self):
        return self.pos_ctrl.plot()

    def read_ctrl_pos(self):
        return self.pos_ctrl.read_ctrl_pos()

    def read_ctrl_quat(self):
        return self.rot_ctrl.read_ctrl_quat()

    def read_grip_change(self):
        return self.pos_ctrl.read_grip_change()
