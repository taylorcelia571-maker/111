# 直线提速、弯道保护与实际转向反馈

用户报告：1.8 m/s 五圈约 2 分 26.50 秒（按 146.50 秒理解），
2.2 m/s 在第二次回到起点附近撞墙。这个反馈不支持继续提高全程速度上限。
本版保持当前能跑的 Navfn/TEB 与轮速前馈，新增可关闭的速度调度。
未修改官方模型、赛道、裁判、地图、航点或 PID。

## 模块实际做什么

- 从 TEB 的 `local_plan` 提取物理距离上的曲率，结合制动距离和 0.25 s 反应预算，提前压低目标速度。
- 用规划转角及实际转向关节角中较大的曲率限速，避免规划器已要求回正但车轮仍转弯时立刻加速。
- 直线速度上限与弯道速度上限分开，过渡连续；出弯加速受 1.8 m/s² 的指令加速上限约束。
- 不滤波转向，不修改规划器停车为前进，不给超过 TEB 要求的速度。
- 缺少新鲜的轨迹或转向状态时，降到弯道速度上限；里程计缺失/过期则停车。
- 反向指令先停车；保留原来的超时与急停功能。

默认直线 2.0、弯道 1.8 m/s，弯道值来自用户已报告的可运行档位，不是实测极限。
转角从 0.06 到 0.16 rad 区间渐进收紧速度；紧于基础 0.5 m 半径时，另按曲率压低上限。
这一横向预算由弯道速度和基础半径推导，未测量轮胎摩擦极限。
它是速度保护与前视调度，不是动力学 MPC，也不能保证不会撞墙。

## 更新并重启

在自己运行的 A/B/C 终端 Ctrl+C 停止本轮，保留已有最快有效成绩。

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source devel/setup.bash
rospack find tianracer_gazebo
```

路径必须为 `/home/tianbot/ican_race_ws/src/tianracer/tianracer_gazebo`。
每个新终端都需要 source Noetic 与工作空间，不能只切换目录。

终端 A：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo adaptive_race_demo.launch straight_speed:=2.0 corner_speed:=1.8 corner_tuning:=false
```

终端 B：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_sim_referee referee.launch
```

完成官方环境检测与绑定、车辆停稳，点击准备发车。终端 C：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo waypoint_laps.launch world:=tianracer_racetrack robot_name:=tianracer laps:=5 switch_distance:=0.5
```

不需要录线；C 沿用已有航点流程。不要同时启动连续路线控制或其他指令节点。
`corner_tuning` 首轮关闭；如果此前最快配置开启了它，应在速度调度稳定后单独比较开/关。

## 参数检查与测试顺序

新终端加载环境后检查：

```bash
rosparam get /tianracer/nav_sim/corner_speed_control
rosparam get /tianracer/nav_sim/straight_speed
rosparam get /tianracer/nav_sim/corner_speed
rosparam get /tianracer/move_base/TebLocalPlannerROS/max_vel_x
```

应是 true、2.0、1.8、2.0。旧的 smooth_race_demo 不会自动开启本模块。

先验证三次有效五圈，尤其观察第二次回到起点附近。
稳定后只提高直线值到 2.2，弯道仍 1.8：

```bash
roslaunch tianracer_gazebo adaptive_race_demo.launch straight_speed:=2.2 corner_speed:=1.8 corner_tuning:=false
```

这与之前“全程 max_speed:=2.2”不同，但仍需实测，不能保证此前撞墙点完全解决。
如果起点附近仍擦墙，先降 `corner_speed:=1.6`，检查轨迹和反馈，不继续提高直线或弯道速度。
如果输出转角本身不合理，速度调度不能修复错误路线，需要保留日志进一步分析。
不要随意扩大换点距离跳过该区域；官方五圈必须通过全部检测线。

比较三轮的中位数、最快值和完赛率，以 146.50 秒作为目前用户报告的新基准。
本版尚未跑过你的 Gazebo 赛道，可能在弯多的路线更慢；不能承诺“大幅度”降低时间。

## 看懂限速日志和报告

终端 A 每两秒输出 `requested / output / limit / reason`。

- `straight`：当前与前方没有更低的弯道约束。
- `current_turn`：当前指令或实际车轮还在转弯。
- `preview_turn`：前方弯道需要提前减速。
- `feedback_fallback`：轨迹或转向状态不新鲜，未确认直线，限在弯道档位。
- `odom_stale`：里程计缺失/超过 0.35 仿真秒，输出零速并告警。
- `planner_stop`：规划器要求停车；本模块不会擅自开车。

查看数据是否存在：

```bash
rostopic hz /tianracer/move_base/TebLocalPlannerROS/local_plan
rostopic hz /tianracer/joint_states
rostopic hz /tianracer/odom
```

如果始终反馈回退或停车，不要靠加速掩盖缺失数据。检查新终端加载的包、topic 和时间戳。

跑圈期间另一个已加载环境的终端：

```bash
rosrun tianracer_gazebo race_diagnostics.py __ns:=/tianracer _duration:=60 _output:=$HOME/ican_race_logs/adaptive_report.json
```

新版报告记录速度调度参数，并区分：

- 规划指令速度 `command_speed_mean_m_s`；
- 送到底盘的限速后指令 `actuator_speed_mean_m_s`；
- 实际车速 `actual_speed_mean_m_s`；
- 模块主动降速 `planner_to_actuator_reduction_mean_m_s`；
- 底盘相对输出指令的欠速 `speed_deficit_mean_m_s`。

这样不会把主动限速误判为底盘执行失败。若仍慢或撞墙，提供报告、该区域的 A 日志和官方五圈结果。

## 验证与回退

本次总计 65 项离线测试通过：之前 50 项，加速度调度 8 项、转换器分支 3 项、
主动限速统计 1 项、启动参数传递 3 项。
涵盖提前限速、实际转向未回正、零速保留、反向制动、反馈缺失和旧模式隔离。
ROS 测试仍用假消息，没有在云端验证插件、物理侧滑、官方模型资格或实际圈速。

关闭：停止本轮并重启此前能完成五圈的 `smooth_race_demo.launch max_speed:=1.8 ...`。
本模块在旧入口默认关闭，地图、模型、裁判和 PID 不需要恢复。
