"""
This module contains the :class:`Config` class.
"""

import io

import numpy as np
import yaml

from classic_framework.utils.sim_path import sim_framework_path


class Config:
    """
    Class contains helper functions to:
    - save a file as .npy file
    - load a .npy file
    - save a file as .yaml
    - load a .yaml file
    - return a list of keys and values as dictionary
    """

    def __init__(self, config_dir):
        """
        Initializes configuration directory.

        :param config_dir: configuration directory
        """
        self.config_dir = config_dir

    def save2npy_file(self, file, name_of_file, path2be_saved=None):
        """
        save file to a .npy file

        :param file: the obj to be saved
        :param name_of_file: the name of the .npy file
        :param path2be_saved: save file to certain directory, if none then save to current project's directory
        :return:
        """
        if path2be_saved is None:
            path2be_saved = self.config_dir
        np.save(sim_framework_path(path2be_saved, name_of_file), file)

    def load_npy_file(self, file_name, path_of_file=None):
        """
        load .npy file. path_of_file does not include the file name

        :param file_name: 'file.npy'
        :param path_of_file: path to the file, but it does not include the file name itself
        :return: the numpy obj
        """
        if path_of_file is None:
            path_of_file = self.config_dir
        return np.load(sim_framework_path(path_of_file, file_name))

    def save2yaml(self, data, name_of_file, path2be_saved=None):
        """
        Method saves data as yaml file to path2be_saved

        :param data: dictionary to be saved
        :param name_of_file: name of the file without .yaml
        :param path2be_saved: path to save the file
        :return:
        """
        if path2be_saved is None:
            path2be_saved = self.config_dir
        with io.open(sim_framework_path(path2be_saved, name_of_file + '.yaml'), 'w') as stream:
            yaml.dump(data, stream)

    def load_yaml(self, file_name, path_of_file=None):
        """
        load .yaml file. path_of_file does not include the file name

        :param file_name: file name without .yaml ending
        :param path_of_file: path to the file, but it does not include the file name itself
        :return: the dictionary
        """
        if path_of_file is None:
            path_of_file = self.config_dir

        print(path_of_file)
        print(file_name + '.yaml')
        with open(sim_framework_path(path_of_file, file_name + '.yaml'), 'r') as stream:
            data = yaml.safe_load(stream)
        return data

    def write2dict(self, list_elems, list_name_keys):
        """
        Writes elements of a list with a given key set to a dictionary.

        :param list_elems: list; Dict values
        :param list_name_keys: list; Dict keys
        :return: dictionary
        """
        data = {}
        for i in range(len(list_elems)):
            elem = list_elems[i]
            if type(elem) is np.ndarray:
                elem = elem.tolist()
            key_elem = list_name_keys[i]
            data[key_elem] = elem
        return data
