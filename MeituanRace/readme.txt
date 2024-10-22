这是一个基于python的简单demo，不包含复杂的算法设计和效率提升，是对竞赛SDK的简易流程示教，demo可以一直送完配置文件中的订单量。
使用流程如下：
// 进入root目录
cd ~
// 创建ros空间,进入并初始化
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
// 创建ros包
catkin_create_pkg race_demo roscpp rospy std_msgs
// 将race demo移动到包目录下
cp /home/sdk_for_user/race_demo/race_demo.tar.xz ~/catkin_ws/src/
// 解压并删除压缩包
tar -xf race_demo.tar.xz
rm race_demo.tar.xz

// 编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash

// 运行
rosrun race_demo demo.py
