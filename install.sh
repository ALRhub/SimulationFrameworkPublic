#!/bin/bash

############ GENERAL ENV SETUP ############
echo New Environment Name:
read envname

echo Creating new conda environment $envname
conda create -n $envname python=3.10 -y -q

eval "$(conda shell.bash hook)"
conda activate $envname

echo
echo Activating $envname
if [[ "$CONDA_DEFAULT_ENV" != "$envname" ]]
then
    echo Failed to activate conda environment.
    exit 1
fi

### Set Channel vars
conda config --add channels conda-forge
conda config --set channel_priority strict


############ PYTHON ############
echo Install mamba
conda install mamba -c conda-forge -y -q


############ REQUIRED DEPENDENCIES (PYBULLET) ############
echo Installing dependencies...
mamba install -c conda-forge pybullet pyyaml scipy opencv pinocchio matplotlib gin-config gym -y -q

# Open3D for PointClouds and its dependencies. Why does it not install them directly?
mamba install -c conda-forge scikit-learn addict pandas plyfile tqdm -y -q
# conda version is badly out of date
#mamba install -c open3d-admin open3d -y -q

# Pre-Commit for Code Style Conventions
echo Installing Pre-Commit
mamba install -c conda-forge pre-commit -y -q
cd `dirname "$BASH_SOURCE"` && pre-commit install


############ MUJOCO BETA SUPPORT INSTALLATION ############
function mj_beta_install() {
  mamba install -c conda-forge imageio -y -q
  pip install mujoco
}

# Mujoco Install Switch
echo
PS3="Do you wish to install the new Mujoco > 2.1 Support? "
select yn in "Yes" "No"; do
    case $yn in
        Yes )
            mj_beta_install
            break
            ;;
        No )
            break
            ;;
    esac
done


############ MUJOCO SUPPORT INSTALLATION ############
function mujoco_install () {
    # Mujoco System Dependencies
    mamba install -c conda-forge glew patchelf -y -q

    # Set Conda Env Variables
    conda env config vars set LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco210/bin:/usr/lib/nvidia
    conda env config vars set LD_PRELOAD=$LD_PRELOAD:$CONDA_PREFIX/lib/libGLEW.so

    # Install MujocoPy
    pip install mujoco-py
}

# Mujoco Install Switch
echo
PS3="Do you wish to also install Mujoco 2.1 Support? (Legacy) "
select yn in "Yes" "No"; do
    case $yn in
        Yes )
            mujoco_install
            break
            ;;
        No ) 
            break
            ;;
    esac
done

echo Installing Open3D
pip install open3d

############ INSTALL ALR-SIM & FINALIZE ############
echo
echo Installing ALR-Sim Package
cd `dirname "$BASH_SOURCE"` && pip install -e .

echo
echo Successfully installed alr-sim

exit 0
