# FAST_LIO_LC_LOCALIZATION

## Dependency
1. ros-noetic
2. Eigen 3.3.4
3. PCL 1.10.0 (noetic defalt)
4. gtsam 4.0.0 alpha2
5. numpy 1.17.4
6. open3d 0.10.0.0
7. ceres-solver 2.2.0
   
### EIGEN
```
wget -O Eigen.zip https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.zip
unzip Eigen.zip
cd eigen-3.3.4/
mkdir build
cd build
cmake ..
make -j2
sudo make install
```

### PCL

```
sudo apt-get install libpcl-dev
sudo apt-get install ros-noetic-pcl-conversions ros-noetic-pcl-ros
```

### GTSAM

```
wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
cd ~/Downloads/gtsam-4.0.0-alpha2/
mkdir build
cd build
cmake -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install
```

### Ceres-Solver

```
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libsuitesparse-dev
git clone https://github.com/ceres-solver/ceres-solver.git
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j3
make test
sudo make install
```

### Numpy

```
sudo apt-get install ros-noetic-ros-numpy
```

### Open3d

```
pip3 install open3d==0.10.0.0
```

## How to use (Case velodyne16)

1. Turn on Lidar and IMU node

2. Make a map with loop-closure

```
roslaunch fast_lio mapping_velodyne.launch
```

3. Localization

```
roslaunch fast_lio localization_velodyne.launch
```

## Reference

FAST_LIO_LC

https://github.com/yanliang-wang/FAST_LIO_LC


FAST_LIO_LOCALIZATION

https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION
