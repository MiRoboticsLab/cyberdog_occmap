# cyberdog_occmap

# 1. 简介
cyberdog_occmap基于输入的位置和姿态实时生成相应栅格地图，基于查表更新和Protobuf等技术，实现了非常轻量化的实时栅格建图算法。

# 2. 程序依赖
cyberdog_occmap在**Ubuntu 20.04**和**ROS2 galactic**上进行了测试，其他环境运行可能会遇到一些依赖问题。

## ROS2依赖
cyberdog_occmap是一个ROS2程序，依赖`geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `message_filters`, `std_srvs`等ROS2相关库，采用以下方式进行安装：
```bash
sudo apt install ros-galactic-geometry-msgs ros-galactic-nav2-msgs ros-galactic-sensor-msgs ros-galactic-std-msgs ros-galactic-message-filters ros-galactic-std-srvs -y
```

## Boost
cyberdog_occmap依赖[Boost](https://www.boost.org/)处理文件的读取、写入和删除，基于以下方式进行安装：
```bash
sudo apt install libboost-dev
```

## Eigen3
cyberdog_occmap依赖[Eigen](http://eigen.tuxfamily.org)进行矩阵运算，采用以下方式安装：
```bash
sudo apt install libeigen3-dev
```

## OpenCV4
cyberdog_occmap依赖[OpenCV](http://opencv.org)对发布后的栅格地图进行去噪和美化处理，使用ROS2自带的OpenCV版本即可。

## Yaml-cpp
cyberdog_occmap依赖[Yaml-cpp](https://github.com/jbeder/yaml-cpp)读取参数文件，采用以下方式安装即可：
```bash
sudo apt install libyaml-cpp-dev
```

## Ceres
cyberdog_occmap依赖[Ceres](http://ceres-solver.org/)对位置姿态进行二次优化，要求**Ceres为源码编译安装**，版本1.14.0，基于Ceres官方提供方式进行编译安装[Ceres-install](http://ceres-solver.org/installation.html)

## Protobuf
cyberdog_occmap依赖[Protobuf](https://github.com/protocolbuffers/protobuf)对栅格地图的子图进行序列化存储，从而大大节省了内存和处理器的使用。要求Protobuf为源码编译安装，版本为3.19.2

## OCCMAP
cyberdog_occmap是在occmap基础之上编写的ROS2接口库，为方便代码和接口分离，此程序已将occmap放置在3rdparty内，无需开发者手动编译。但开发者如需更改occmap程序，需要在更改后对occmap依照以下方式进行重新编译安装，即可使得更改内容生效。
```bash
cd 3rdparty/occmap
bash install.sh
```

## 其他依赖
cyberdog_occmap依赖[cyberdog_visions_interfaces]()和[cyberdog_common]()来与其他cyberdog程序进行交互，开发者如需单独使用cyberdog_occmap，需要对接口进行重写。