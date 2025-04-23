# iiwa_kuka_kinematics
Solution of forvard and inverse kinematics problems for the manipulator KUKA IIWA 

# System

Ubuntu 22.04

# Requirements

## 1. Eigen

```bash
sudo apt install cmake libboost-dev libblas-dev liblapack-dev

wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz

gunzip eigen-3.4.0.tar.gz

cd eigen-3.4.0

mkdir build && cd build

cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release

cmake --build .

sudo make install
```

## 2. nlohman json

```bash
sudo apt-get -y install nlohmann-json3-dev
```

## 3. KDL

```bash
git clone https://github.com/orocos/orocos_kinematics_dynamics

cd orocos_kinematics_dynamics/orocos_kdl

mkdir build && cd build

cmake .. 

cmake --build .

sudo make install
```

## 4. Pinocchio

```bash
git clone https://github.com/stack-of-tasks/pinocchio.git

cd pinocchio

mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF

cmake --build .

sudo make install
```

## 5. Drake

```bash
sudo snap install curl

curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg

sudo mv bazel-archive-keyring.gpg /usr/share/keyrings/

echo "deb [signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list

sudo apt update

sudo apt install -y bazel

git clone https://github.com/RobotLocomotion/drake.git

cd drake

mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release

cmake --build .

sudo make install
```
