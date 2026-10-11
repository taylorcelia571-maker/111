# 弯道轮速前馈与可选 TEB 小步调优

本版针对用户报告的过弯卡顿、慢速和摆动，先修正底盘执行，再单独比较局部规划参数。
不要求录线，仍使用原航点和 Navfn/TEB，官方模型、赛道、地图、裁判、物理参数和 PID 配置均未改动。
云端没有 ROS/Gazebo，下面是待本机实测的改动，不是已经证明的最快配置。

## 已定位的问题与尚未证实的瓶颈

| 项目 | 代码证据及处理 |
| --- | --- |
| 指令执行开销 | 原 servo_commands 每条指令创建六个发布器；现在只初始化一次 |
| 弯道轮速 | 原四轮同速；加入可关闭的阿克曼内外轮、前后轮滚动速度前馈 |
| 局部预瞄/频率 | 可选 TEB 20 Hz、0.2 s 时间步长、4 m 预瞄；原 Navfn 和避障权重保留 |
| 按全局路线走 | 实验性的低权重路径参考点单独开关，默认关闭；不再强制高权重追逐折线路径 |
| 速度瓶颈 | TEB 同时受 max_vel_x、max_vel_theta、转弯半径和优化约束限制，不能只增大 max_vel_x |
| 直线/弯道策略 | 直线速度上限仍为 max_speed，弯道由 TEB 运动学和角速度预算决定，轮速前馈按当前转角变化 |
| 转向延迟/振荡 | 诊断新增实际转向关节与指令对比、延迟估计、转角步长和变化率 |
| 动态 PID | 保持现有 P/D，先测关节响应；没有证据时自动加大 PID 可能引入振荡 |
| 预测/曲率 | TEB 继续优化前方轨迹，并保留最小转弯半径约束；本次未加入新的动力学 MPC |

例如半径 0.5 m 弯道、角速度预算 3.6 rad/s，对应约 1.8 m/s 的运动学速度预算。
增加直线速度上限并不意味着这种紧弯也能同速。可选 TEB 配置将角速度预算小幅改为 4.0，
但实际允许的速度还取决于轨迹、避障、加速度和轮胎滑移，不能把这个例子视为保证。

轮速前馈使用原 URDF 几何：轴距 0.26 m、后轮距 0.18 m、前转向铰链距 0.13 m、
轮子相对铰链偏移 0.025 m、轮半径 0.032 m。它是稳态滚动几何前馈，
不保证转舵瞬态、滑移或动力学条件下完全无误差。中心转角限制 0.5 rad，以保留模型铰链 ±0.6 rad 限制。
关闭轮速前馈时保留原轮速与原 0.265 m 转角转换，方便基线对比。

## 更新与启动

你的 `waypoint_laps.launch` 是终端 C 的航点发送程序，不决定速度和局部规划模式。
新的配置通过终端 A 启用。先停止自己启动的旧仿真、裁判和跑圈，再更新、构建：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source devel/setup.bash
rospack find tianracer_gazebo
```

包必须来自 `/home/tianbot/ican_race_ws/src/tianracer/tianracer_gazebo`。
每个新终端都 source Noetic 和 `~/ican_race_ws/devel/setup.bash`。

**先只试轮速前馈，速度保持 1.8，不同时调其他参数。** 终端 A：

```bash
roslaunch tianracer_gazebo smooth_race_demo.launch max_speed:=1.8 wheel_feedforward:=true corner_tuning:=false path_bias:=false
```

终端 B：

```bash
roslaunch tianracer_sim_referee referee.launch
```

官方环境检测和绑定正常、车辆停稳后准备发车。终端 C：

```bash
roslaunch tianracer_gazebo waypoint_laps.launch world:=tianracer_racetrack robot_name:=tianracer laps:=5 switch_distance:=0.5
```

这个 C 命令沿用你现有流程。官方五圈与导航闭环序列仍独立，以裁判最终锁定为准。

## 分步寻找更快且稳定的配置

同一速度下，每档至少三次有效五圈，并记录失败轮次，不只选择最快一次。

1. 在上述 A 命令中改 `wheel_feedforward:=false`，测基线；true 测轮速前馈。
2. 若 true 更顺、更稳定，再改 `corner_tuning:=true` 测局部规划调优，保持速度 1.8。
3. 有效完赛稳定后，分别试 `max_speed:=2.0`、2.2、2.4；每次重新准备轮次。
4. `path_bias:=true` 仅作为后续单独实验。它是软路径参考，不是强制沿折线行驶；
   若局部规划失败、停顿加重或轨迹不合理，关闭它，不继续提高权重。

组合示例（完成前两步确认后才用）：

```bash
roslaunch tianracer_gazebo smooth_race_demo.launch max_speed:=2.0 wheel_feedforward:=true corner_tuning:=true path_bias:=false
```

可选局部参数在 `tianracer_gazebo/config/teb/corner_tuning.yaml`。
代价地图膨胀、未知区域处理、障碍/运动学权重、迭代数及最小转弯半径没有变化，
避免再次组合第三版导致首目标规划失败的配置。全部开关默认可恢复关闭。

## 在跑圈期间量化瓶颈

另开已加载环境的终端，在车辆正常跑圈时执行（不是等跑圈退出后）：

```bash
rosrun tianracer_gazebo race_diagnostics.py __ns:=/tianracer _duration:=60 _output:=$HOME/ican_race_logs/corner_report.json
```

若 rosrun 指向旧工作空间，可直接运行新源码：

```bash
python3 ~/ican_race_ws/src/tianracer/tianracer_gazebo/scripts/race_diagnostics.py __ns:=/tianracer _duration:=60 _output:=$HOME/ican_race_logs/corner_report.json
```

脚本只读，不控制车辆；采集 60 个墙钟秒，统计使用仿真时间。
终端打印汇总，完整采样保存 JSON。它同时记录实际 planner 参数，避免只提供 C 命令时无法确认配置。

- `straight/corner.command_speed_mean_m_s`：判断规划器是否主动在弯道降低速度。
- `actual_speed_mean_m_s`、`speed_deficit_mean_m_s`：判断底盘是否跟不上指令；欠速还可能来自碰撞或滑移。
- `speed_ceiling_fraction`：大量时间达到上限时，直线速度上限可能是瓶颈。
- `yaw_budget_fraction`：大量时间接近角速度预算时，弯道速度受运动学预算影响。
- `steering_lag_estimate_s`、`steering_tracking_rmse_rad`：在有充分转角变化和关节状态时估计转向响应。
- `steering_step_p95_rad`、`steering_rate_rms_rad_s`：量化转角跳变。
- `steering_trend_reversals`：转角变化趋势反转次数；真实连续弯也会导致反转，不能直接当作振荡次数。

直线/弯道采用小转角和较大转角阈值分类，仅是诊断近似；停车等待也会计入。
延迟估计只在 0..0.3 s 范围内匹配，不是自动系统辨识或 PID 最优值。
仿真时钟重置时统计作废；不同参数应分别保存报告。

如果要调整 PID，应先在相同路线/速度下确认实际转向滞后或过冲，再单独比较 P/D；
不能同时调速度、轮速前馈和 PID。本版没有自动改变控制器增益，也没有绕过官方校验。
将报告和各档官方五圈用时一起提供后，可以对瓶颈给出基于数据的结论。

## 验证和回退

本次共 50 项离线测试通过：此前控制/恢复检查 34 项，轮速与执行器生命周期 9 项，
统计与延迟估计 4 项，可选局部参数和启动传递检查 3 项。
检查包含轮速符号、左右镜像、刚体滚动约束、URDF 几何、模型转角限制、发布器只初始化一次和急停。
ROS 适配测试仍使用假消息；云端没有 ROS/Gazebo，未实测五圈提升。

```bash
cd ~/ican_race_ws/src/tianracer
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_wheel_feedforward.py' -v
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_motion_metrics.py' -v
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_corner_profile.py' -v
```

回退：在 A 中关闭 `wheel_feedforward`、`corner_tuning` 和 `path_bias`，速度恢复 1.8。
发布器一次初始化的修正仍保留。或直接回到 `planner_race_demo.launch max_speed:=1.8`。
原地图、车模、裁判和 PID 文件均未改动。
