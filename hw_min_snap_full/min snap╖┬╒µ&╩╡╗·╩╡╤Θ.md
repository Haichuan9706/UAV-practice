# min snap仿真&实机实验

### 仿真运行方法

1. 编译 `catkin_make`后source `source devel/setup.bash`
2. 运行仿真 `roslaunch min_snap min_snap.launch`，可以看到rviz内出现一架悬停飞机
3. **设定3D goal方法**：按一下键盘`G`键，按住鼠标左键，可以看到一个简单，此时按住左键不松，同时按住右键，移动鼠标可以看到箭头上下移动
4. 设定航点：拖动出若干个z轴高度大于0的3D goal，最后拖动出一个z轴高度小于0的3D goal，可以看到规划出一条轨迹，无人机开始跟随
5. 可设置参数：`min_snap.launch`内的`mean_vel`参数，决定了飞机飞行的平均数据
6. 
### 代码编写
在成功跑通仿真后，将min_snap下的`min_snap_closeform.cpp`替换为src同级下的`min_snap_closeform.cpp`，并进行填空，由于本实验较难，大家可以先尝试着自行填空，觉得被block了可以参考一下原先的正确代码，但不要一上来就看答案。
### 实机注意事项

1. workspace下已集成px4ctrl，当然你可以替换成自己曾经写的那份
2. 修改ekf_pose下launch的动捕topic
3. 如果有将7通道设置为kill switch的组，请在地面站中取消，因为7通道需要使用

### 飞行方法
1. 确保通道7拨杆在远离飞手的位置（朝外）

2. `sh min_snap.sh`

3. 像悬停实验一样将飞机在auto hover下悬停

4. 将7通道拨杆拨向靠近飞手的位置（朝内），此时飞机进入程序控制模式，且保持悬停

5. 像仿真一样点出航点，出现轨迹后无人机会沿着轨迹飞行

6. 降落方法：将7通道拨杆拨向外，此时回到auto hover模式，接着像悬停实验一样将飞机降落

