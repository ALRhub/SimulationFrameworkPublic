import numpy as np


def has_collision(obj1_name, obj2_name, sim):
    obj1 = sim.model.geom_name2id(obj1_name)
    obj2 = sim.model.geom_name2id(obj2_name)
    for i in range(sim.data.ncon):
        contact = sim.data.contact[i]
        if contact.geom1 == obj1 and contact.geom2 == obj2:
            return True
        elif contact.geom1 == obj2 and contact.geom2 == obj1:
            return True
    return False


def change_domain(value, in_low, in_high, out_low, out_high):
    return (out_high - out_low) * ((value - in_low) / (in_high - in_low)) + out_low


def obj_position(sim, obj_id: str):
    return sim.data.get_body_xpos(obj_id)


def obj_distance(sim, obj1: str, obj2: str):
    obj1_pos = obj_position(sim, obj1)
    obj2_pos = obj_position(sim, obj2)
    dist = np.linalg.norm(obj1_pos - obj2_pos)
    rel_dist = (obj1_pos - obj2_pos) / dist + 1e-8
    return dist, rel_dist
