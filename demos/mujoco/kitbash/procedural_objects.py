import xml.etree.ElementTree as Et

import numpy as np

from classic_framework.mujoco.mujoco_utils.mujoco_scene_object import MujocoLoadable


class KitBashContainer(MujocoLoadable):
    """
    Procedurally generated Object consisting of multiple box geoms.
    It iss a graph-like structure.
    """
    counter = 0

    def __init__(self, max_children=6):
        """
        Args:
            max_children: max number of boxes the kitbash object consists of
        """
        # Random initialization
        self.pos = np.append(np.random.random_integers(-10, 10, 2) / 10, np.random.randint(1, 4))
        self.name = 'kitbash{}'.format(KitBashContainer.counter)
        self.euler = np.random.uniform(size=3)
        KitBashContainer.counter += 1

        # Generate child geoms
        self.children = []
        children = np.random.randint(2, max_children)
        self.grow(children)

    def grow(self, children: int):
        """
        Recursevily attach boxes to the KitBash object
        Args:
            children: number of child objects to attach

        Returns:
            None
        """
        # Anchor
        if children < 1:
            return

        # Create 'Root' Node
        if len(self.children) < 1:
            self.children.append(KitBashNode())
        # Attach new nodes
        else:
            # Find a node to attach to. If already 4 nodes are attached, try the next one.
            # Worst case: The last node (highest idx) will be the youngest with guaranteed free sides.
            parent_idx = np.random.randint(len(self.children))
            while np.sum(self.children[parent_idx].sides) >= 4:
                parent_idx += 1
            parent = self.children[parent_idx]
            # Child Box handles how it appends to the parent.
            self.children.append(KitBashNode(parent))

        # Recursive Call
        return self.grow(children - 1)

    def to_xml(self, scene_dir: str):
        object_body = Et.Element('body')
        object_body.set('name', self.name)
        object_body.set('pos', ' '.join(map(str, self.pos)))
        object_body.set('euler', ' '.join(map(str, self.euler)))
        freejoint = Et.SubElement(object_body, 'freejoint')

        # get geoms from child nodes
        for c in self.children:
            object_body.append(c.to_geom())

        return object_body, False


class KitBashNode:
    def __init__(self, parent=None):
        """
        Args:
            parent: parent KitBashNode object. Defaults to None.
        """

        # Can be initialized nearly always safely at random
        self.size = np.random.random_integers(1, 4, 3) / 50
        self.rgba = np.append(np.random.rand(3), 1)
        self.sides = np.zeros(6)

        # Check if it is the 'root' node. The root is always positioned at [0, 0, 0,]
        if parent is None:
            self.pos = np.zeros(3)
        else:
            # Choose and attach to a side of both parent and child (self)
            parent_side = parent.pick_side()
            child_side = (parent_side + 3) % 6
            parent.attach(parent_side)
            self.attach(child_side)

            # To avoid self intersection, move center of child node away from parent side in proportion to its new size.
            self.pos = parent.side_pos(parent_side)
            offset = np.zeros(3)
            _, axis = self.get_side_characteristics(child_side)
            offset[axis] = np.sign(self.pos[axis]) * self.size[axis]
            self.pos = np.add(self.pos, offset)

    def attach(self, side: int):
        """
        increase "bitmask" to indicate free sides.
        Args:
            side: int from 0-6 indicating the side of the bounding box
        """
        self.sides[side] += 1

    def get_side_characteristics(self, side):
        """
        Find the axis and direction of the side
        Args:
            side: int from 0-6 indicating the side of the bounding box

        Returns:
            sign, axis --> sign is the direction in -1/+1, axis is the index corresponding to XYZ
        """
        sign = -1 ** (side > 2)
        axis = side % 3
        return sign, axis

    def side_pos(self, side: int):
        """
        choose a position on the given side
        Args:
            side: int from 0-6 indicating the side of the bounding box

        Returns:
            pos: 3 dimensional position array
        """
        sign, axis = self.get_side_characteristics(side)
        # Choose a random point in the bounding box
        pos = [np.random.uniform(- self.size[i], self.size[i]) for i in range(3)]

        # clamp the chosen axis in its `sign` direction to transform the point to be on the outer bounding box
        pos[axis] = sign * self.size[axis]
        return pos

    def pick_side(self):
        """
        choose a random, free side with uniform probability
        Returns:
            side: int from 0-6 indicating the side of the bounding box
        """
        chosen_side = None
        count = 0
        for side in range(len(self.sides)):
            if self.sides[side] == 0:
                count += 1
                if np.random.randint(count) == 0:
                    chosen_side = side
        return chosen_side

    def to_geom(self):
        """
        transform the KitBashNode to a XML geom node to use in the mujoco scene XML
        Returns:
            geom: XML Element
        """
        geom = Et.Element('geom')
        geom.set('type', 'box')
        geom.set('size', ' '.join(map(str, self.size)))
        geom.set('rgba', ' '.join(map(str, self.rgba)))
        geom.set('pos', ' '.join(map(str, self.pos)))

        return geom
