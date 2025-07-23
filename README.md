# Swarm-SLAM: Sparse Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot Systems <!--![Build Status](https://github.com/MISTLab/Swarm-SLAM/actions/workflows/main.yml/badge.svg)-->

Follow the [start-up instructions](https://lajoiepy.github.io/cslam_documentation/html/md_startup-instructions.html) to install, build and run Swarm-SLAM.

## Swarm-SLAM setup

### System installation:

**Installed on WSl2 Ubuntu-24.04**

ROS2 HUMBLE
```bash
locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo apt update

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop ros-dev-tools 

# Make sure that ROS2 is sourced in each shell
echo "source /opt/ros/humble/setup.bash" >>  ~/.bashrc
```


##### Miniconda install and init
```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
source ~/miniconda3/bin/activate
conda init --all
conda create --name cslam
```

##### Clone repo
```bash
sudo apt install python3-pip python3-vcstool
cd ~
git clone https://github.com/Skuddo/Swarm-SLAM.git
cd Swarm-SLAM
mkdir src
vcs import src < cslam.repos
conda activate cslam
pip install --break-system-packages -r requirements.txt
pip install pybind11
```

##### GTSam
```bash
cd ~
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.1.1
mkdir build
cd build
cmake ..
sudo make install
```

##### TEASER++ python bindings (fixed branch custom repo)
```bash
cd ~
git clone https://github.com/Skuddo/TEASER-plusplus.git
cd TEASER-plusplus 
git checkout develop
mkdir build && cd build
conda activate cslam
cmake .. && make
make teaserpp_python
sudo make install 
```

##### Get ROS2 dependencies
```bash
cd ~/Swarm-SLAM
sudo apt install python3-rosdep python3-colcon-common-extensions
conda activate cslam
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src --rosdistro jazzy
```

##### Build repo
```bash
cd ~/Swarm-SLAM
cd src
conda activate cslam
colcon build

# Make sure the project is sourced in each shell
echo "source ~/Swarm-SLAM/src/install/setup.bash" >>  ~/.bashrc
```

#### Fixes
Add these lines to your .bashrc if wsl keeps adding windows pyenv to your wsl path
Modify the grep argument (pyenv paths) if your pyenv main directory is elsewhere

```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '/mnt/c/Users/.*/.pyenv/pyenv-win/shims' | paste -sd ':' -)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '/mnt/c/Users/.*/.pyenv/pyenv-win/bin' | paste -sd ':' -)
```

### Replication

>> T1 - CSLAM run 1
```bash
ros2 launch cslam_experiments kitti_stereo.launch.py robot_id:=0 bag_start_delay:=10.0
```
>> T3 - visualize
```bash
ros2 launch cslam_visualization visualization.launch.py
```

## Other repos used

Packages summary:
- [cslam](https://github.com/lajoiepy/cslam): contains the Swarm-SLAM nodes;
- [cslam_interfaces](https://github.com/lajoiepy/cslam_interfaces): contains the custom ROS 2 messages;
- [cslam_experiments](https://github.com/Skuddo/cslam_experiments): contains examples of launch files and configurations for different setups;
- [cslam_visualization](https://github.com/lajoiepy/cslam_visualization): contains an online (optional) visualization tool to run on your base station to monitor the mapping progress.
- [GTSAM](https://github.com/borglab/gtsam): Smoothhing and mapping lib
- [TEASER++](https://github.com/Skuddo/TEASER-plusplus): package for lidar points (custom)
- [CosPlace](https://github.com/gmberton/CosPlacen): NN model for images


