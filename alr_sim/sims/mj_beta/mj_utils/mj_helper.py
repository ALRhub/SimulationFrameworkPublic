import logging
from enum import Enum

import mujoco
import numpy as np


class IncludeType(Enum):
    FILE_INCLUDE = 0
    MJ_INCLUDE = 1
    WORLD_BODY = 2
    VIRTUAL_INCLUDE = 3  # This type suppresses the load operation.


def get_mj_geom_id(obj_name, mj_model):
    obj_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_GEOM, obj_name)

    if obj_id == -1:
        logging.getLogger(__name__).warning(
            "Could not find geom for obj1 with name {}".format(obj_name)
        )

    return obj_id


def has_collision(mj_scene, obj1_name: str, obj2_name: str = None) -> bool:
    """Check if a collision with obj1 occured.
    If obj2_name is unspecified, any collision will count, else only collisions between the two specified objects.

    Args:
        mj_scene (_type_): Mujoco Scene
        obj1_name (str): name of the geom to be checked
        obj2_name (str, optional): name of a second geom. If undefined, all collisions are considered valid. Defaults to None.

    Returns:
        bool: true if a collision occured, false otherwise
    """
    obj1 = get_mj_geom_id(obj1_name, mj_scene.model)
    if obj1 == -1:
        return False

    if obj2_name is not None:
        obj2 = get_mj_geom_id(obj2_name, mj_scene.model)
        if obj2 == -1:
            return False

    for i in range(mj_scene.data.ncon):
        contact = mj_scene.data.contact[i]
        if contact.geom1 == obj1 or contact.geom2 == obj1:
            if obj2_name is None:
                return True
            elif contact.geom1 == obj2 or contact.geom2 == obj2:
                return True
    return False


def change_domain(value, in_low, in_high, out_low, out_high):
    return (out_high - out_low) * ((value - in_low) / (in_high - in_low)) + out_low


def obj_position(data, obj_id: str):
    return data.get_body_xpos(obj_id)


def obj_distance(data, obj1: str, obj2: str):
    obj1_pos = obj_position(data, obj1)
    obj2_pos = obj_position(data, obj2)
    dist = np.linalg.norm(obj1_pos - obj2_pos)
    rel_dist = (obj1_pos - obj2_pos) / dist + 1e-8
    return dist, rel_dist


def reset_mocap2body_xpos(model, data):
    """Resets the position and orientation of the mocap bodies to the same
    values as the bodies they're welded to.
    """

    if model.eq_type is None or model.eq_obj1id is None or model.eq_obj2id is None:
        return
    for eq_type, obj1_id, obj2_id in zip(
        model.eq_type, model.eq_obj1id, model.eq_obj2id
    ):
        if eq_type != mujoco.const.EQ_WELD:
            continue

        mocap_id = model.body_mocapid[obj1_id]
        if mocap_id != -1:
            # obj1 is the mocap, obj2 is the welded body
            body_idx = obj2_id
        else:
            # obj2 is the mocap, obj1 is the welded body
            mocap_id = model.body_mocapid[obj2_id]
            body_idx = obj1_id

        assert mocap_id != -1
        data.mocap_pos[mocap_id][:] = data.body_xpos[body_idx]
        data.mocap_quat[mocap_id][:] = data.body_xquat[body_idx]


def get_body_jacr(model, data, name):
    id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, None, jacr, body=id)
    return jacr


def get_body_jacp(model, data, name):
    id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, None, body=id)
    return jacp


def get_body_xvelp(model, data, name):
    jacp = get_body_jacp(model, data, name).reshape((3, model.nv))
    xvelp = np.dot(jacp, data.qvel.copy())
    return xvelp


def get_body_xvelr(model, data, name):
    jacr = get_body_jacr(model, data, name).reshape((3, model.nv))
    xvelr = np.dot(jacr, data.qvel.copy())
    return xvelr
