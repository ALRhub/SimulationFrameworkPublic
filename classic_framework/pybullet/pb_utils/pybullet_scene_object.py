import os

from classic_framework.utils.sim_path import sim_framework_path


class PyBulletObject:
    def __init__(self,
                 urdf_name,
                 object_name,
                 position,
                 orientation,
                 data_dir=None):

        if data_dir is None:
            data_dir = sim_framework_path('./envs/data/')

        objects = os.listdir(data_dir)

        if not urdf_name.endswith('.urdf'):
            if urdf_name + '.urdf' in objects:
                urdf_name += '.urdf'
            else:
                raise ValueError("Error, object with name " + urdf_name + " not found. Check that a object with the "
                                                                          "specified name exists in the data "
                                                                          "directory")

        assert len(position) == 3, "Error, <position> has three entries x, y, z."
        assert len(orientation) == 3, "Error, <orientation> has three entries yaw, pitch, and roll."

        self.__urdf_name = urdf_name
        self.__object_name = object_name
        self.__position = position
        self.__orientation = orientation
        self.__data_dir = data_dir

    @property
    def urdf_name(self):
        return self.__urdf_name

    @property
    def object_name(self):
        return self.__object_name

    @property
    def position(self):
        return self.__position

    @property
    def orientation(self):
        return self.__orientation

    @property
    def data_dir(self):
        return self.__data_dir
