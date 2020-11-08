import mujoco_py


def get_body_to_idx_dict(model):
    body_to_idx = {}
    for body_name in model.body_names:
        body_to_idx[body_name] = model.body_names.index(body_name)
    return body_to_idx


def get_geom_to_idx_dict(model):
    geom_to_idx = {}
    for geom_name in model.geom_names:
        geom_to_idx[geom_name] = model.geom_names.index(geom_name)
    return geom_to_idx


def get_joint_to_idx_dict(model):
    joint_to_idx = {}
    for joint_name in model.joint_names:
        joint_to_idx[joint_name] = model.joint_names.index(joint_name)
    return joint_to_idx


def get_site_to_idx_dict(model):
    site_to_idx = {}
    for site_name in model.site_names:
        site_to_idx[site_name] = model.site_names.index(site_name)
    return site_to_idx


def get_actuator_to_idx_dict(model):
    actuator_to_idx = {}
    for actuator_name in model.actuator_name:
        actuator_to_idx[actuator_name] = model.body_names.index(actuator_name)
    return actuator_to_idx


def reset_mocap2body_xpos(sim):
    """Resets the position and orientation of the mocap bodies to the same
    values as the bodies they're welded to.
    """

    if (sim.model.eq_type is None or
            sim.model.eq_obj1id is None or
            sim.model.eq_obj2id is None):
        return
    for eq_type, obj1_id, obj2_id in zip(sim.model.eq_type,
                                         sim.model.eq_obj1id,
                                         sim.model.eq_obj2id):
        if eq_type != mujoco_py.const.EQ_WELD:
            continue

        mocap_id = sim.model.body_mocapid[obj1_id]
        if mocap_id != -1:
            # obj1 is the mocap, obj2 is the welded body
            body_idx = obj2_id
        else:
            # obj2 is the mocap, obj1 is the welded body
            mocap_id = sim.model.body_mocapid[obj2_id]
            body_idx = obj1_id

        assert (mocap_id != -1)
        sim.data.mocap_pos[mocap_id][:] = sim.data.body_xpos[body_idx]
        sim.data.mocap_quat[mocap_id][:] = sim.data.body_xquat[body_idx]
