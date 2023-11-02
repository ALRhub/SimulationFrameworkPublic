from setuptools import setup, find_packages

setup(
    name="alr_sim",
    version="0.2",
    description="Franka Panda Simulators",
    license="MIT",
    author="Autonomous Learning Robots @ KIT",
    url="https://alr.anthropomatik.kit.edu/",
    package_data={"models": ["*"]},
    packages=find_packages(),
)
