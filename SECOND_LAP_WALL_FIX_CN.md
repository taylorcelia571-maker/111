# 第二圈停车：墙距余量修正

## 本次日志确定的原因

本轮使用 optimized_race、curvature_feedback、max_speed=2.2、response_delay=0.25。第一参考圈 24.534 秒，官方第一圈 23.969 秒；约 1.09 个参考闭环、仿真时刻 250.628 秒发生：

```text
Race stopped: Localized vehicle too close to static wall
raw_rear_pose=(-0.30400395, 3.79559727, 1.99224897)
tf_age_seconds=0.01
```

该车姿的车辆中心为 (-0.35718518, 3.91422170)，仓库地图复算净空 0.22762009 m，低于 0.23 m 停车阈值。因此控制器主动停车并保持 inactive；不是现有日志证明的 Gazebo 卡死或 TF 超时，也不是参考闭环数量达到上限。截图中裁判随后保存 1 圈、67.104 秒、dnf/aborted，裁判结束本轮本身不控制车辆。

旧候选参考线净空最低只有约 0.26 m，离 0.23 m 墙距阈值仅 3 cm。车辆可以满足 15 cm 参考偏差保护，却先违反墙距保护。本次日志的参考偏差约 5.5 cm，足以耗尽这一余量。日志另有横向里程计速度约 0.169 m/s，仍需实际动力学数据才能判断侧滑与定位误差各自影响，不能据此直接调 PID。

## 代码修正

1. optimized_race 的 `route_margin` 默认提高至 0.34 m：在原版地图内重新选线，给原 0.23 m 停车阈值留出 11 cm 名义余量。不改变停车阈值或地图/车辆模型。若当前地图不能满足净空/转向/航点顺序约束，则规划失败并保持静止。
2. prepare 支持 curvature_feedback 静止预转向，并继续要求新鲜关节、静止车辆和无其他控制发布者。上一版验证文档让这个模式调用只支持 smooth_pursuit 的服务，确实是文档/API 不一致；本次一并修复。旧 prepare=False 不是这次第二圈停车原因，start 后日志显示车辆已正常行驶。
3. start 发车前也检查静态墙距；不能从已靠墙的故障车姿重新起步。运行期间靠墙仍锁定停车，错误增加实际净空、阈值、中心坐标和后轴车姿，便于核查。
4. 移除验证流程中额外读取和录制 `/gazebo/model_states` 的命令。你的模型在订阅者从 1 增到 2 时明确打印禁止使用提示；这里只让独立裁判读取它，车辆控制只用地图、定位、里程计、雷达和关节反馈。通过裁判 GUI 确认真实赛道/车辆，不再使用 echo 或 rosbag 订阅真值。该警告与日志中的墙距停车属于两个不同检查。

新路线会更宽、更保守，可能增加圈时。本次先修复完赛稳定性，不能宣称已经达到五圈 60 秒。

## 更新并重新生成起点

先结束失败轮、Ctrl+C 停止旧控制终端与仿真。不要从当前停车位置重新 start。更新：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

包路径必须指向该工作空间，更新或构建失败不能继续使用旧版本。每个新终端都执行两行 source。

终端 A：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

终端 B：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_sim_referee referee.launch
```

终端 C，保持运行，等待 Ready：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo optimized_race.launch
```

终端 D 检查新参数和预转向：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosparam get /tianracer/continuous_race/route_margin
rosparam get /tianracer/continuous_race/lateral_control
rosservice call /tianracer/continuous_race/prepare '{}'
```

应显示 0.34、curvature_feedback、prepare success=True；Ready 的中心参考线最低净空应不小于 0.34 m。车辆应仍静止。确认没有旧 rosbag 或 echo 正在订阅 model_states，也没有其他控制器运行。在裁判确认官方环境/绑定并“准备发车”后调用：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

观察至少第二圈同一弯道，再跑到裁判有效五圈。完赛后：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
rostopic echo -n 1 /tianracer/odom/twist/twist
```

若再次停住，保持控制终端 C 打开，先保存报错，调用：

```bash
rosservice call /tianracer/continuous_race/status '{}'
cat ~/ican_race_logs/control_run_report.json
```

status 返回 active、last_fault、最后反馈与控制原因；不要通过反复 start 覆盖故障。保留控制日志和裁判事件，之后按前述步骤重建起点。只提供 C 完整日志与 status 就能先判断原因，不必订阅 Gazebo 真值。

## 验证与限制

增加日志车姿的数值回归：原阈值下必定复现 22.76 cm 的墙距停车；新增整条候选线检查，在 ±8 cm 横向扰动及 ±0.08 rad 航向扰动组合下，车辆中心地图净空仍高于原阈值。新路线在假设 80/150/250 ms 执行器响应模型中完成五个参考闭环，保留墙距和跟踪偏差保护。还验证 curvature_feedback 的 prepare 保持零速度并保留关节检查、靠墙拒绝 start、行驶中靠墙仍锁定停车。

这些是离线回归，不能替代 ROS2GO 实际动力学和官方有效过线。当前没有设备实测，不能保证这次必然完成五圈；需要按上述流程确认三次有效完赛后再考虑提速。最新离线报告为 [SECOND_LAP_OFFLINE_RESULTS.json](SECOND_LAP_OFFLINE_RESULTS.json)，旧 RACE60 报告保留为上一版历史证据。
