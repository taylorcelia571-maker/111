# 连续控制中途停车：本次修正与验证

本次日志的停车原因是 `Tracking error exceeds safe corridor`，不是 Gazebo 卡死。入弯前指令为 1.10 m/s，里程计仍为 3.18 m/s；最后参考线偏差达到 0.1513 m，超过 0.150 m 的保护阈值，控制器锁定停车。最后 TF 年龄 0.018 秒，这次没有证据表明 TF 超时是原因。仅提高 max_speed、反复调用 start 或取消停车保护无法解决跟踪失稳。

新增响应时间预留：每周期按实测速度计算提前距离，读取此距离内的最低速度包络，提前发出减速指令。距离跨圈时按闭合路径连续计算，不重置起步状态。`response_delay=0.25` 是保守调参预留，不是已测得的电机响应时间；状态服务返回 `preview_distance`、`preview_speed`、`speed_excess`。实测速度比指令高 0.5 m/s 以上时另输出警告。

激光制动检查也使用至少相同的响应时间。报告中的模型圈时不包含在线提前限速的额外耗时，不可用作这套配置的预计正式成绩。

`finish_race.launch` 现使用 max_speed=2.2、lateral_accel=3.2、acceleration=1.5、braking=2.0、yaw_rate=2.5，关闭按时间选线、路径细调、姿态外推和起步连接线。这是恢复完赛的配置，不是争取一分钟的配置。保留原来的激光、地图墙距、数据时效和 15 cm 偏差保护；不自动从故障位置重新起步。官方地图、车辆和裁判未修改。

## 左右摆动的横向控制修正

后续日志确认测试仍使用 rapid_race 的 8.0/6.5/5.0 组合，出现约 0.475 rad 航向误差及转角输出跟不上纠偏需求。日志中的 steer 是指令角，不是实际关节角，不能仅凭这一列测出舵机响应延迟。

新完赛配置启用 `lateral_control=curvature_feedback`，使用路径曲率前馈、连续航向误差反馈和随实测速率调整的横向误差反馈。它不再在偏离时突然缩短追踪目标距离来产生急剧纠偏，也将纠偏限速改为连续计算。读取 `/tianracer/joint_states` 中左右转向关节，换算等效转角；当实际角度和目标角度不一致时限制速度，防止转向未就位就加速。

此模式需要新鲜、有效的关节反馈；缺失、超过 300 ms 或非有限值会拒绝起步/停车，禁止沿用无效数据。旧 pursuit 模式作为比较基线保留在 planned/rapid 入口。**按本页启动 finish_race；继续执行旧 rapid 命令不会启用新的横向控制器。** 这次没有改车辆 PID，当前没有同步测量证明 PID 本身是摆动原因。

## 更新并重新准备一轮

先在裁判结束当前失败轮。Ctrl+C 停止旧导航和旧仿真；不要同时保留 waypoint_laps、move_base 导航、rapid_race 或其他速度发布程序。更新：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

最后一行必须指向 `~/ican_race_ws/src/tianracer/tianracer_gazebo`。每个新终端都加载上述两行 source；直接执行 setup.bash 或只 cd 到目录不能切换工作空间。

终端 A，重新生成官方场景，让车辆回到默认静止起点：

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

终端 C，先规划，等待 `Ready`：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo finish_race.launch
```

启动后应看到 `Controller: curvature_feedback; speed budget 2.20m/s; steering joint feedback required=True`。在终端 D 检查：

```bash
rosparam get /tianracer/continuous_race/lateral_control
rosparam get /tianracer/continuous_race/max_speed
```

分别应为 `curvature_feedback`、`2.2`。如果仍是 pursuit 或 8.0，则运行的不是本次完赛配置；先停止旧控制器并检查 rospack 路径。

保持 C 运行。在裁判重新检测官方环境、完成绑定并点击“准备发车”。终端 D：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosservice call /tianracer/continuous_race/start "{}"
```

车辆必须仍在起点静止，服务应返回 success: True。不要在上一轮故障位置直接重发 start；偏离或方向错误会继续被拒绝。若服务不存在，说明终端 C 的节点未运行，应读取该终端错误。

官方五圈锁定后停止车辆：

```bash
rosservice call /tianracer/continuous_race/stop "{}"
```

## 验收与限制

以裁判的有效 5/5、实际五圈时间为验收标准。先连续验证三轮完赛，再分别调一个速度预算；本次不再使用 8.0/6.5/5.0 的高预算组合作为恢复配置。发生停车时保存控制终端完整日志、`planned_course_report.json`，并运行下面的状态服务；它会显示最后的实测速率和限制原因。

```bash
rosservice call /tianracer/continuous_race/status "{}"
```

开发检查包含真实地图几何，以及假设 80、150、250 ms 一阶速度/转向响应的五圈自行车模型，直接读取 finish_race 的默认速度参数、模拟关节实测反馈；另检查定位扰动、纠偏切换处的连续性和关节反馈失效。这些检查不倒车、保持墙距、跟踪偏差和跨圈不重新起步，不能证明 Gazebo 中不会侧滑、撞墙或停车。云环境没有 ROS/Gazebo，本次未完成真实仿真五圈验证，也未验证一分钟目标。
