# 无需录线的连续闭环导航

针对本次 141.134 秒五圈成绩及 TEB infeasible 重置，增加一套独立的连续控制算法。旧 TEB 导航仍可用于对照。新入口不启动 move_base，也不运行 waypoint_laps；它不属于修改 TEB 二进制，而是替换点到点的局部导航控制流程。

## 算法改变

- 静止时从当前 ROS 地图和原来的四个有序引导航点生成整圈路线。A* 障碍距离代价优先选择宽裕的走廊，未知格视为障碍。
- 周期平滑覆盖首尾接缝；每个候选检查完整线段的障碍距离、转向曲率和航点顺序。规划失败就不启动运动。不是直接用直线连接四个航点。
- 后轴参考的速度自适应预瞄跟踪，50 Hz 输出，限制转向变化率。轮速前馈处理内外轮速度差。没有倒车纠偏或每圈重新发送起点目标。
- `v <= sqrt(lateral_accel / |curvature|)`，同时约束航向变化率、转向变化率，并在闭环上做向后制动/向前加速传播。直线和紧弯共用连续的速度包络，接缝不是零速边界。
- 激光检查沿预规划弯道的走廊，避免将当前转角无限外推到弯道出口的墙上；另有静态地图距离、跟踪误差、传感器时效、控制超时保护。出现故障会锁定停止，需要排查并重新启动。

`lateral_accel` 是控制预算，不是已经测出的侧翻极限。当前地图路线最紧曲率约 1.7/m，预算 5 m/s² 对应约 1.7 m/s 紧弯速度；直线可以达到 `max_speed`。极限还受轮胎摩擦、重心、转向响应和定位误差影响，不能仅凭圈速算出“保证不翻车”的最大速度。暂不改原车轮 PID 或官方模型。

## 更新并确认代码路径

停止旧仿真、旧 waypoint_laps 和其他控制节点，重新启动整套环境。每个新终端都加载工作空间。不要直接执行 setup.bash，也不要只 cd 到源码目录来切换 ROS 包。

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
sudo apt-get update
sudo apt-get install -y python3-scipy python3-numpy python3-yaml
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack profile
rospack find tianracer_gazebo
ls "$(rospack find tianracer_gazebo)/launch/planned_race_demo.launch"
ls "$(rospack find tianracer_gazebo)/scripts/planned_race.py"
```

`rospack find` 应指向 `/home/tianbot/ican_race_ws/src/tianracer/tianracer_gazebo`。指向 tianbot_ws 表示加载了其他工作空间，先修正环境再启动。

## 运行顺序

终端 A：官方赛道、车辆、定位及底盘控制。不发送导航任务。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

终端 B：裁判。检测、绑定和正式环境校验按赛事教程进行。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_sim_referee referee.launch
```

终端 C：第一次用 1.8 验证路径跟踪，成功后再重跑 2.0，最后按实测考虑 2.1。提高直线速度时先保持横向加速度预算不变，不要再直接把所有弯道速度抬到 2.2。

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo planned_race.launch max_speed:=1.8 lateral_accel:=5.0
```

等待 `Ready` 和 `Idle`，此时车辆仍静止。RViz 添加 Path 显示 `/tianracer/race_path` 可检查整圈路线。计划保存到 `~/ican_race_logs/planned_course.csv`，列为后轴 x、y、曲率、速度上限，不能当作裁判结果。

裁判点击“准备发车”后，终端 D 发车：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosservice call /tianracer/continuous_race/start "{}"
```

必须收到 `success: True`；否则按返回信息处理，车辆不会自动倒车寻找路线。**本流程不再启动 waypoint_laps.launch。** 同时存在其他 cmd_vel 发布者会拒绝启动。

裁判锁定五圈后，立即停止车辆：

```bash
rosservice call /tianracer/continuous_race/stop "{}"
```

默认最多行驶六个参考闭环，给官方判圈留出起始位置差异；裁判锁定结果不自动给车辆制动。参考闭环圈数不能代替官方五圈。必须以官方显示 5/5、校验和完赛用时判断效果，再按教程提交。

## 比较及排查

同样的正式赛道、初始位置和定位条件下，每个速度档重复至少三次，比较官方五圈总用时、完赛成功率、碰撞和各圈差异。目标低于 120 秒；尚未在 Gazebo 实测达成。

保留日志：

```bash
mkdir -p ~/ican_race_logs
rosbag record -O ~/ican_race_logs/planned_test.bag /clock /tf /tf_static /tianracer/scan /tianracer/odom /tianracer/cmd_vel /tianracer/ackermann_cmd_stamped /tianracer/race_path
```

若 `Race stopped`，保留原因与日志。传感器失效、定位偏离或墙体遮挡的制动不能通过关闭保护来消除。若仍出现 TEB infeasible 或 GOAL Reached，说明旧 move_base 还在运行，检查 `rosnode list` 并关闭旧启动终端。

## 已完成和未完成的验证

已在仓库赛道地图完成整圈几何检查、周期速度约束、官方初始位置的理想自行车模型五圈连续跟踪、地图模拟激光检查和 ROS 假传输的启动/停止检查。理想模型不包含轮胎侧滑、真实转向延迟、车身倾斜或 AMCL 定位噪声，不能证明实际不撞墙、不翻车或两分钟内完赛。

云端无 ROS/Gazebo，不能执行正式仿真或裁判判圈。官方赛道、地图、航点坐标、车辆模型、底盘 PID 和裁判没有修改。路线平滑能否通过所有官方检测线仍需实际裁判确认。
