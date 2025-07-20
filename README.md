# Swarm-SLAM: Sparse Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot Systems <!--![Build Status](https://github.com/MISTLab/Swarm-SLAM/actions/workflows/main.yml/badge.svg)-->

Follow the [start-up instructions](https://lajoiepy.github.io/cslam_documentation/html/md_startup-instructions.html) to install, build and run Swarm-SLAM.

## Swarm-SLAM Replication

Startup:

Installs
```bash
sudo apt install python3-vcstool
sudo apt install python3-pybind11
sudo apt install python3-rosdep python3-colcon-common-extensions
```   
ROS2
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade
sudo apt install ros-dev-tools ros-jazzy-desktop

echo "source /opt/ros/jazzy/setup.bash" >>  ~/.bashrc
```

Clone repo
```bash
git clone https://github.com/Skuddo/Swarm-SLAM.git
cd Swarm-SLAM
mkdir src
vcs import src < cslam.repos
```

Miniconda
```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
source ~/miniconda3/bin/activate
conda init --all
conda create --name cslam
conda activate cslam
sudo apt install python3-pip
pip install -r requirements.txt 
```

GTSam
```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.1.1
mkdir build
cd build
cmake ..
make install
```

TEASER++
```bash
git clone https://github.com/Skuddo/TEASER-plusplus.git
cd TEASER-plusplus 
git checkout develop
mkdir build && cd build
cmake .. && make
sudo make install 
```

Build
```bash
cd ~/Swarm-SLAM
mkdir src
cd src
conda activate cslam
colcon build
```

Packages summary:
- [cslam](https://github.com/lajoiepy/cslam): contains the Swarm-SLAM nodes;
- [cslam_interfaces](https://github.com/lajoiepy/cslam_interfaces): contains the custom ROS 2 messages;
- [cslam_experiments](https://github.com/lajoiepy/cslam_experiments): contains examples of launch files and configurations for different setups;
- [cslam_visualization](https://github.com/lajoiepy/cslam_visualization): contains an online (optional) visualization tool to run on your base station to monitor the mapping progress.


