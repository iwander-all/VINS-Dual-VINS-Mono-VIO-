# VINS-Dual：基于VINS-Mono的双目VIO系统开发

这个系统是基于香港科技大学飞行机器人组的开源框架VINS-Mono开发的，原开源框架是针对单目SLAM。本双目SLAM系统是在原开源框架基础上的二次深度开发，外部接口与原框架一致。对VINS-Mono的深度分析请见[我的博客](https://blog.csdn.net/iwanderu/article/details/104617829)或support_files，您可以依此对比原开源框架[VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)或[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)与[本系统](https://github.com/iwander-all/VINS-Dual-VINS-Mono-VIO-)的区别。这个项目是我的研究课题项目，非商业用途，感谢HKUST的沈老师课题组提供的开源框架。


<a>下一步迭代计划：</a>
<a>(1)通过yaml文件设置，可以自由选择 左目+IMU，右目+IMU，或双目+IMU 三种VIO形式。</a>
(2)利用极线搜索为左右目光流提供初始值；
(3)
之前有哥们提醒我无法运行，后来发现忘记把config的yaml文件上传上来，现在已经上传。另外如果需要运行回环检测，请将vins-mono的support files里的brief_k10L6.bin放到这个项目的同名目录下，否则无法实现回环检测的功能。

## 1.需要的基本配置
```
Ubuntu 16.04
ROS Kinetic
Ceres Solver
OpenCV 3.3.1
Eigen 3.3.3
```

## 2.在您的ROS上构造VINS-Dual
```
cd ~/catkin_ws/src
git clone https://github.com/iwander-all/VINS-Dual-VINS-Mono-VIO-.git
cd ../
catkin_make
```

## 3. VINS-Dual的启动
请在[EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)下载您需要测试的数据集。

打开1个控制台，输入：
```
roscore
```

进入devel目录，分别打开2个控制台，输入：
```
source setup.bash
```

再分别输入：
```
roslaunch vins_estimator euroc.launch 
roslaunch vins_estimator vins_rviz.launch
```

如果不使用回环检测，请输入：
```
roslaunch vins_estimator euroc_no_posegraph.launch 
```

再打开1个控制台，请输入：
```
rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag 
```

运行效果：[Video](https://www.bilibili.com/video/BV167411m7wR/)。

## 4.VINS-Dual的框架介绍

<img src="https://img-blog.csdnimg.cn/20200323160913371.jpeg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2l3YW5kZXJ1,size_16,color_FFFFFF,t_70"/>

VINS_dual包含两个ROS结点，分别是特征提取结点和vio结点；

**featureTracker node：**

**main线程**：订阅双目视觉帧，放入各自的buf中；

**featureTracker线程**：获取2个buf中的视觉帧，对齐时间戳，分别进行光流跟踪(左目：当前帧与前一帧的光流追踪；右目：右目的当前帧和左目的当前帧的光流追踪)，并发布同一时刻上双目追踪到的全部特征点的像素坐标(发布的特征点左目和右目的信息数量是一样的且**一一对应**)。


**vio node：**

**main线程**：订阅IMU，特征点frame信息，分别放入各自的buf中；

**vio线程**：获取特征点和IMU信息并对齐；IMU预积分；从图像帧中获取信息并给feature类添加/补充新的特征点(的左目和右目对齐的信息)；确定滑窗策略；初始化；后端优化；

初始化过程定义为**class Initial**，负责系统的初始化；
非线性优化定义为**class Backend**,负责滑动窗口和非线性优化。

其中，非线性优化所维护的H矩阵包括：
(1)先验误差；
(2)相邻两帧的IMU误差； 
(3)具有共视点的两帧之间的重投影误差(重投影误差提供了**三种残差策略**：纯左目的重投影误差；纯右目的重投影误差；融合双目的重投影误差。可以通过更改yaml文件中的flag确定重投影误差模式)。


## 5.参考文献
T.Qin, P.L.Li, S.J.Shen, VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator.

## 6.联系方式
rhuag@connect.ust.hk
我将持续对代码进行更新和改进。

