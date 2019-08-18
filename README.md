# VIO Data Simulation

IMU和Camera数据仿真，用于VIO算法测试。

这份代码是<https://github.com/HeYijia/vio_data_simulation>的一个fork。

![demo pic](data/demo.png)

## 坐标系

- **B**ody frame: IMU坐标系

- **C**am frame: 相机坐标系

- **W**orld frame: 第一帧的IMU坐标系

- **N**avigation frame: NED（东北天）或ENU（北东地）,本代码采用的是ENU，重力向量在该坐标系下为$(0,0,-9.81)$

目前，IMU的z轴向上，xy平面内做椭圆运动，z轴做正弦运动，x轴沿着圆周向外。外参数Tbc将相机坐标旋转，使得相机朝向特征点。

## 代码结构

utils/generate_data.cpp：用于生成IMU数据，相机轨迹，特征点像素坐标，特征点的3d坐标

include/paramc.h：IMU噪声参数，IMU频率，相机内参数等等

python/：文件夹里为可视化工具，draw_points.py就是动态绘制相机轨迹和观测到的特征点，依赖`numpy matplotlib`。

## 数据存储的格式

### 特征点

> x, y, z, 1, u, v

每个特征出现在文件里的顺序，就是他们独立的id，可用来检索特征匹配。

### IMU data

> timestamp(1), imu_quaternion(4), imu_position(3), imu_gyro(3), imu_acc(3)

### Camera data

> timestamp(1), cam_quaternion(4), cam_position(3), imu_gyro(3), imu_acc(3)

注意，由于IMU和Camera的存储采用的是同一个函数，所以Camera data也会存储一些`gyro, acc`这些数据，但是没用，是多余存储的。

## Simulation for direct tracking

To cope with direct tracking, image data instead of distinct point should be generated. Thus we modify the project to generate corresponding frames.
