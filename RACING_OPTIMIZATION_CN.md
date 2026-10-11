# iCAN 2026 TEB 跑圈优化使用说明

根据用户提供的参赛规则，沿用 ROS1 Noetic / Gazebo 11 / Python 3.8 的
TEB Demo 与多航点链路。没有改变裁判、检测线、成绩文件、赛道、车辆模型或物理参数。
网页在本次环境中返回 403；规则依据是用户粘贴的参赛文档。

本包是基于上传 ZIP 的代码修改，不是官方发布版本。
正式环境仍须使用 `tianbot/tianracer` 的赛事发布代码（文档指定 `dev`），
赛道 `tianracer_racetrack` 版本 `2026.09.1`，并通过官方模型校验。
本次未核实上传包的官方版本资格，不要用修改包替换官方裁判或模型。

## 安装

先停止自己的导航和仿真进程，备份当前源码和本地修改。
`tianracer-race-overlay.zip` 只含本次改动，根目录为 `tianracer-main/`。
将其中对应文件合并到当前赛事仓库，勿替换其他文件。
完整包 `tianracer-optimized.zip` 用于查看全部代码；比赛安装优先使用改动包。
`race-changes.diff` 可用于逐项审阅。已有本地修改时先人工合并。

在工作空间构建并重新加载：

```bash
source /opt/ros/noetic/setup.bash
cd ~/tianbot_ws
catkin_make
source devel/setup.bash
chmod +x "$(rospack find tianracer_gazebo)/scripts/transform.py"
chmod +x "$(rospack find tianracer_gazebo)/scripts/waypoint_laps.py"
```

`race_motion.py` 必须和 `transform.py` 同在 `scripts/`，不能只复制后者。
现有工程使用源码空间入口；不要把单个脚本复制到其他位置执行。

## 运行

每个终端先加载 Noetic 和工作空间环境。终端 A：

```bash
roslaunch tianracer_gazebo demo_tianracer_teb_nav.launch world:=tianracer_racetrack robot_name:=tianracer race_profile:=true race_max_speed:=2.4
```

终端 B：

```bash
roslaunch tianracer_sim_referee referee.launch
```

保持车辆静止、检测官方赛道与车辆、完成赛事绑定，点击“准备发车”。
仅在裁判提示等待发车后，终端 C 启动控制：

```bash
roslaunch tianracer_gazebo waypoint_laps.launch world:=tianracer_racetrack robot_name:=tianracer laps:=5 switch_distance:=0.5
```

五次闭合航点序列不保证是五个官方有效圈。观察裁判是否依次通过全部检测线并锁定五圈结果。
若导航已结束但裁判不足五圈，检查路线和起点，不要修改本地结果补圈。
裁判锁定结果后在提交窗口内点击提交，并确认平台受理。
用户提供的本届截止时间为 2026-10-15 23:59 北京时间。

## 修改与调参

| 改动 | 作用 |
| --- | --- |
| 修正默认频率的覆盖顺序，基础配置 5 → 10 Hz | 避免 launch 的 10 Hz 被 YAML 覆盖成 5 Hz |
| 可选 race profile 20 Hz、50 Hz 控制输出 | 更频繁规划，输出随时间连续变化；不等于 50 Hz 重规划 |
| 轨迹时间步长 0.30 → 0.15 s，预瞄 3 → 4.5 m | 给入弯留出规划距离 |
| 轴距 0.265 m、精确弧长 | 与现有仿真舵机转换使用的轴距一致 |
| 直线速度上限 1.8 → 2.4 m/s | 提供较快的测试起点；不保证能稳定完赛 |
| 曲率速度约束 `v <= sqrt(a_lat / abs(tan(delta)/L))` | 转弯时降低指令速度，出弯受残余转角约束后再加速 |
| 转向低通与变化率限制、加速限制 | 减少转角跳变和突然加速 |
| 零速立即执行、0.5 仿真秒指令超时 | 停止命令不等待减速滤波；中断后不持续使用旧指令 |

基础最小转弯半径、车体轮廓和障碍间距保持原值。
`config/teb/race_profile.yaml` 控制规划参数；`config/race_motion.yaml` 控制输出平滑参数。
速度上限通过 `race_max_speed` 同时传给规划器和控制器。
`Twist.angular.z` 在这条链路中是转向角，必须保留 `cmd_angle_instead_rotvel: true`。

先用 `race_max_speed:=1.8` 确认五圈路线与过弯，再试 2.0、2.2、2.4。
每档至少记录三次官方五圈总用时、有效完赛次数，以及贴墙或漏线情况。
速度、曲率阈值是调参初值，不是赛事上限，也不是已经测出的最优值。
若入弯反应滞后，优先降低速度或缩小 `steer_tau`，不要继续提高速度。
`steer_tau` 默认 0.06 s；`steer_rate` 默认 1.8 rad/s。
曲率限速对突变目标可立即压低速度，优先于舒适减速；它不是轮胎动力学保证或障碍检测器。
平滑会改变 TEB 的实际执行轨迹，因此仍需观察安全间距与规划器追踪误差。
换点距离仍为 0.5 m，没有为了提速任意扩大或修改默认四航点。
发现漏线、抄近路时先按官方赛道重新录制足够的航点。

检查实际参数：

```bash
rosparam get /tianracer/move_base/controller_frequency
rosparam get /tianracer/move_base/TebLocalPlannerROS/max_vel_x
rosparam get /tianracer/nav_sim/race_profile
rostopic hz /tianracer/cmd_vel
rostopic hz /tianracer/ackermann_cmd_stamped
```

正常仿真运行时，目标频率分别为 20 Hz 与 50 Hz；暂停或运行负载会影响实际频率。
若 TEB 无法达到 20 Hz，先降低速度、检查计算负载。
关闭可选优化：重启 A 时传 `race_profile:=false`。
这会恢复原来的直接指令转换与基础 TEB 参数，但基础频率已修正为 10 Hz。
需要完整原版时恢复备份中的所有被修改文件。

## 本次验证范围

本次在 Debian / Python 3.12 环境执行 12 项离线测试，全部通过：
速度上限、转向连续性与变化率、入弯/出弯曲率速度约束、零速与异常指令、
倒车切换、指令超时、仿真时钟重置、关闭优化及退出停车。
转换器测试使用假消息与假时间，不代替真实 ROS 通信验证。
修改过的 launch/XML、YAML 合并与参数传递已检查。
新增/改动 Python 源码按 Python 3.8 语法检查。

```bash
cd ~/tianbot_ws/src/tianracer
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_race_*.py' -v
```

本次环境没有 ROS/Gazebo，未执行 catkin 构建、车辆仿真、官方裁判或真实圈速对比；
原有依赖 ROS 消息的 `test_waypoint_laps.py` 也未运行。
不能据此宣称已减少多少秒或已通过比赛环境校验。
