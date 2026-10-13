# 起步纠偏、里程计修复与实际 Gazebo 对照

最新用户官方五圈成绩为 **92.921 秒**，首圈 21.965 秒，后四圈平均 17.739 秒。首圈多出的约 4.23 秒值得优化，但即使全部消除，仍约 88.70 秒，不能仅靠起步达到一分钟。附件实际使用 steering_feedback_gain=0，不能把这次变慢归因于已启用转向补偿。

## 修复和新的启动入口

`gazebo_odometry.py` 原回调先把 Gazebo 世界坐标 Twist 放入共享变量，再转换为车身坐标；发布定时器可能在转换完成前读取，混合新位姿、世界坐标速度和旧时间戳。沿反方向直道时，这会把世界坐标的负速度误报为车身倒车；世界坐标横向速度也可能被误报为前进速度，干扰预瞄和限速。确定性线程交错测试复现了这个代码缺陷。修改为在局部副本完成线速度、角速度转换后，原子更新完整快照；TF 和 Odometry 使用同一快照。车辆尚未生成时不再发布带新时间戳的默认身份位姿。保留原来 20 Hz 发布频率和真实倒车保护，不篡改车辆状态。

实际旧版复现曾在第一圈触发“倒车”停车；由于旧日志没有记录触发瞬间的具体速度，不能断言该次停车全部由并发缺陷造成。新版错误日志增加纵向速度和反馈值，方便区分真实倒车与反馈异常。

新增 `quick_start_race.launch`：从静止起点用既有 C2 连续连接轨迹平滑并入参考线，避免先强制纠正约 7 厘米偏差、再左右转向纠偏。起步上限 1.8 m/s、加速度 2.0 m/s²，起步加速度保护保持至前 5 米。连接轨迹不符合起点方向或地图净空时退回原对齐流程，不取消保护。只在起步使用，后续每圈不再执行起步对齐。正常预算保持 10/6.5/5.5/3/3.8；保持 0.34 m 路线净空和 0.25 s 响应预留。入口默认补偿关闭，可明确传入已对照的 gain=0.25。

`rapid_race.launch` 新增并转发 laps、run_report，允许按每次试验保存报告；修正起始偏差报告原先记录为零的问题。

## 实际动力学结果

Ubuntu 20.04、ROS Noetic、Python 3.8.10、Gazebo 11.15.1，无显示窗口；使用仓库原版赛道、车辆、轮胎、PID 和 API 插件。只使用公开 scan、odom、imu、joint_states 反馈控制与观测。每组重新启动场景、恢复默认静止起点，prepare 就绪后 start；速度预算与参考路线相同。**每个配置仅一次试验；下表是控制器参考闭环，不是官方裁判成绩。**

| 配置 | 首个参考闭环 | 五个参考闭环 | 最大参考偏差 | 保护停车 |
| --- | ---: | ---: | ---: | --- |
| 修复里程计，原对齐起步 | 23.130 s | 96.630 s | 0.1322 m | 无 |
| 平滑并线起步，gain=0 | 19.985 s | 94.231 s | 0.1114 m | 无 |
| 平滑并线起步，gain=0.25 | 19.894 s | 92.679 s | 0.1135 m | 无 |

平滑并线首圈减少约 3.15 秒；补偿组相对基准总时间减少约 3.95 秒。整体收益有限，单次差异不能作为稳定收益保证。并线组、补偿组的公开 IMU 没有出现翻车姿态，公开里程计没有低于 -0.1 m/s 的倒车样本；这不保证其他机器或提高预算后也安全。依旧有转向失配和速度响应限速，没有证明“所有弯道不卡顿”。

完整报告：[GAZEBO_STARTUP_RESULTS.json](GAZEBO_STARTUP_RESULTS.json)。非 ROS 算法与适配器回归共 201 项；航点的 11 项测试另外在真实 Noetic 消息依赖下执行。线程交错、真实倒车保持、车身角速度转换和未生成车辆不发布假里程计均有回归测试。

## 为什么调 max_speed 不提速，为什么没有承诺 60 秒

本次路线 45.065 米，计划峰值只有约 6.04 m/s，max_speed=10 已不限制它；增至 11 不会改变计划。相同预算下，不含起步和速度响应预瞄的五圈包络模型约 77.34 秒，加入名义响应预瞄的模型约 87.24 秒。模型不是物理极限认证，但足以说明仅消除起步和增大速度上限没有证据达到 60 秒。

要在这条路线五圈低于 60 秒，平均速度需超过约 3.76 m/s；当前紧弯常在约 2 m/s，仍受转向响应、横向加速度和加减速约束。此前 lateral_accel=6.8 已发生保护停车，不能把预算继续上调当作已证实安全的提速。此次没有跳过规定检测线、修改模型、放宽墙距/偏差/传感器时效保护，也没有修改成绩。**一分钟目标尚未达成。**

## 本机完整验证流程

先停止控制并结束上一轮，退出旧场景和裁判，再更新。每行单独执行；setup.bash 要 source，不能当程序执行。

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
ls ~/ican_race_ws/src/tianracer/tianracer_gazebo/launch/quick_start_race.launch
```

rospack 必须指向 `~/ican_race_ws/src/tianracer/tianracer_gazebo`；如果指向 tianbot_ws，当前测试的代码仍是另一份。**需要重启终端 A 的仿真，才会加载新的里程计脚本。仅重启控制器不够。**

每个新终端均先分别运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
```

终端 A：启动原版官方场景和原始控制桥，不另外启动 waypoint_laps 或 move_base 控制任务。

```bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

终端 B：

```bash
roslaunch tianracer_sim_referee referee.launch
```

确认官方赛道/车辆环境校验有效；正式测试需确认绑定和上报条件有效。保持默认起点静止；本轮开始后不能重置或移动赛道/车辆。

终端 C，等待 Ready；第一轮用补偿关闭的并线起步：

```bash
roslaunch tianracer_gazebo quick_start_race.launch steering_feedback_gain:=0 run_report:=$HOME/ican_race_logs/quick_start_gain0.json
```

终端 D，在点击裁判“准备发车”之前执行：

```bash
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}'
```

等 status 中 steering_ready=true 且车辆仍静止，再点击裁判“准备发车”，然后执行：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

控制日志应出现 `mode: connector`，起步后一段使用 startup_connector，随后 launch_limited=false。如果退回 alignment，看 start_mode 的原因，不能把退回后的时间当作并线方案效果。start=True 只代表接受发车，不代表已经成功完赛。

**以裁判有效五圈锁定为准**；默认六个参考闭环仅给裁判留出有效过线余量，不能用本地参考圈数补官方圈数。官方锁定后立即停止：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
cat ~/ican_race_logs/quick_start_gain0.json
cat ~/ican_race_logs/planned_course_report.json
```

下一轮重新启动场景恢复默认静止起点、重新准备裁判，终端 C 换成：

```bash
roslaunch tianracer_gazebo quick_start_race.launch steering_feedback_gain:=0.25 run_report:=$HOME/ican_race_logs/quick_start_gain025.json
```

按同样 prepare/status/裁判准备/start 顺序复验。两档各三轮，比较有效完赛率、官方总用时中位数、首圈与后四圈、最大偏差和保护停车。新方案若更慢或保护停车，保存当轮报告再回退 gain=0；不继续升高横向预算。不要覆盖上一轮数据或反复 start 覆盖故障证据。

需要同步运动证据时，另开已加载环境的终端，在 start 前录公开消息：

```bash
mkdir -p ~/ican_race_logs
rosbag record -O ~/ican_race_logs/quick_start_trial.bag /clock /tf /tf_static /tianracer/scan /tianracer/odom /tianracer/imu /tianracer/joint_states /tianracer/cmd_vel /tianracer/ackermann_cmd_stamped
```

完赛后 Ctrl+C 结束录制；每轮更换文件名。Connection refused 表示 ROS master/场景未运行或 ROS_MASTER_URI 错误，不能在关掉 A 后录制。
