# from setuptools import setup, find_packages
# import sys
#
# flags = sys.argv
#
# if 'mujoco' or '-m' in flags:
#    with open('req_without_mujoco.txt') as req:
#       requirements = req.read().splitlines()
# else:
#    with open('req.txt') as req:
#        requirements = req.read().splitlines()
#
# setup(
#    name='ALRSim',
#    version='0.1',
#    description='Panda Simulation for Pybullet and Mujoco',
#    license="MIT",
#    author='Autonomous Learning Robots @ KIT',
#    url="https://alr.anthropomatik.kit.edu/",
#    packages=find_packages(),
#    install_requires=requirements
# )




# from setuptools import setup, find_packages
# from setuptools.command.install import install
#
# class InstallCommand(install):
#    user_options = install.user_options + [
#       ('mujoco', None, None),  # a 'flag' option
#    ]
#
#    def initialize_options(self):
#       install.initialize_options(self)
#       self.mujoco = None
#
#    def finalize_options(self):
#       # print("value of someopt is", self.someopt)
#       install.finalize_options(self)
#
#    def run(self):
#       global mujoco
#       mujoco = self.mujoco  # will be 1 or None
#       print("valueof mujoco is", mujoco)
#       from subprocess import call
#       if mujoco is None:
#           req = 'req_without_mujoco.txt'
#       else:
#           req = 'req.txt'
#       call(["pip install -r " + req], shell=True)
#       install.run(self)
#
# setup(
#       cmdclass={'install': InstallCommand},
#       name='ALRSim',
#       version='0.1',
#       description='Panda Simulation for Pybullet and Mujoco',
#       license="MIT",
#       author='Autonomous Learning Robots @ KIT',
#       url="https://alr.anthropomatik.kit.edu/",
#       packages=find_packages()
#       )

from setuptools import setup, find_packages

setup(
      name='ALRSim',
      version='0.1',
      description='Panda Simulation for Pybullet and Mujoco',
      license="MIT",
      author='Autonomous Learning Robots @ KIT',
      url="https://alr.anthropomatik.kit.edu/",
      packages=find_packages()
      )