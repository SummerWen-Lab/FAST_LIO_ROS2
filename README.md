### up70算法组培训task1代码(ROS2+Docker版本) version:20250906 
[注意]该版docker容器已封装成镜像上传dockerhub，调用：
```
FROM summerwen612/fast_lio_wxm:250906-stable
```


**拉取后代码未编译，需先docker-compose build后进入容器，按以下步骤操作：** \
1.编译安装Livox-SDK：
```bash
cd src/Livox-SDK2/build
cmake .. && make -j
sudo make install
```
2.编译安装Livox-ros-driver2:
```bash
cd src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```
3.编译fast_lio和slam_runner(自定义启动包)
```bash
# in /workspace
colcon build
```
4.source一下使新写入配置生效（**注意：每一次打开终端都要重新source**）
```bash
# in /workspace
source /opt/ros/humble/setup.sh
source install/setup.bash
```
**[注意]（坑）这一版为了适配25RC场馆录制的数据 在fast_lio/config/mid360.yaml中修改了lidar_type=4(适配一般PCL2数据),同时修改了fast_lio/config/avia.yaml中的lidar_type=2(不清楚当时录制的雷达型号所以选了个看起来效果最好的），后续如果更换雷达设备这两个参数必须对应修改，不然会出问题（大概率rviz显示不出点云）**

5.运行启动
```bash
ros2 launch slam_runner slam.launch.py
# 可以在运行时通过以下代码监听并生成tf树（.pdf格式）
ros2 run tf2_tools view_frames
```
