# 根据实际日志修正卡顿：TEB 可行性与到点朝向

本轮截图：官方五圈 **160.474 秒**，各圈 33.517、30.874、32.613、31.199、32.271 秒。
此前用户报告的最好档位约 146.50 秒；本轮退化约 13.97 秒。
本次修改基于用户日志，不继续把全程上限提高到已撞墙的 2.2 m/s。

## 日志证据

- 出现 50 次 `trajectory is not feasible. Resetting planner`。需要改善局部轨迹可行性，不能屏蔽碰撞检查。
- 89 条低频采样的速度调度日志中，31 条已有足够速度上限，却仍被额外加速缓升压低。
- 采样请求/输出均值约 1.487 / 1.304 m/s；这些是每两秒一次的采样，不是完整时间加权均速。
- 只有两次 `GOAL Reached`，对应初始到点和最终到点，并非代码每圈都等待 SUCCEEDED。
  中间航点的固定朝向仍会影响 TEB 接近目标时的轨迹和速度，冲线卡顿不能仅归因于显式停车。

## 本版修改

1. 新 `flow_race_demo.launch` 不启用上一版外部速度调度和旧低通，直接执行 TEB 输出，保留轮速前馈和超时停车。
2. 旧 CornerSpeed 类默认也取消重复加速缓升；其原有限速仍可作为可选旧模式，不能覆盖 TEB 零速。
3. TEB 使用 0.15 s 时间步长、4×3 优化迭代、精确弧长和匹配车模的 0.26 m 轴距。
   障碍惩罚权重从 10 到 20，提高对不可行轨迹的优化压力；不是硬性保证。
4. 保留原足迹、最小障碍间距和最小转弯半径。可行性检测 5 个更细时间样本，
   时间跨度不小于基础配置，不关闭或跳过碰撞检查。
5. 到点朝向可根据相邻航点之间的实际全局路径末端切线预先计算，避免任意固定朝向导致临近目标重整姿态。
   航点坐标和顺序不变，空路径、错误帧或查询失败时沿用原朝向。
6. 查询仅在发送第一个导航目标前执行一次。已核对 Noetic 源码：make_plan 服务拒绝活动导航目标；
   跑圈中不调用这个服务，不阻塞每次换点。查询不自动清空代价地图。
7. 诊断报告新增 TEB 不可行重置计数，用于检验改动是否有效。

这仍是 Navfn + TEB，没有改变官方车型、地图、赛道、裁判、物理参数或 PID。
真急弯必须满足转弯和抓地能力，不能保证全程同速；目标是减少重置/重复约束带来的额外停顿。

## 更新并重启

Ctrl+C 停止自己上一轮运行的 A/B/C，避免旧的活动导航目标干扰预规划。

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source devel/setup.bash
rospack find tianracer_gazebo
```

应来自 `~/ican_race_ws/src/tianracer/`。每个新终端都重新加载 Noetic 和工作空间。

终端 A：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo flow_race_demo.launch max_speed:=1.8 flow_tuning:=true
```

终端 B：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_sim_referee referee.launch
```

官方环境和绑定有效、车辆停稳后准备发车。终端 C：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo waypoint_laps.launch world:=tianracer_racetrack robot_name:=tianracer laps:=5 switch_distance:=0.5 align_goal_yaw:=true
```

C 启动后先查询四段路径，此时尚未发送行驶目标，通常需要等待片刻。
成功后日志打印 `uses global-path arrival heading`。
如果打印 `arrival heading unavailable`，保存详细错误；不要通过手动删除碰撞约束来强行成功。
保持原换点距离 0.5，首点和最终闭环仍必须导航成功，失败不会跳过计圈。
导航闭环仍不等同于官方五圈，以裁判锁定结果为准。

## 分辨哪个改动有效

先在 1.8 m/s 测一轮，检查能否有效完赛、冲线附近姿态及重置频率。
若局部调优使规划失败或计算负载过高，A 改 `flow_tuning:=false`，保留 C 的朝向预规划单独对比。
若新朝向导致路线不合理，C 改 `align_goal_yaw:=false`，可单独对比 TEB 参数。
初次不要把这些新模块与旧 corner_tuning、path_bias、速度调度同时叠加。

稳定有效后只把 `max_speed` 提到 1.9，再到 2.0。2.2 已有撞墙记录，不直接跳回该档。
每档至少三次比较官方五圈中位数、最快值、有效完赛率和 TEB 重置频率。
目标是恢复并超过 146.50 秒档位，但当前没有本机实测收益，不能承诺显著减少多少秒。

运行时检查：

```bash
rosparam get /tianracer/nav_sim/corner_speed_control
rosparam get /tianracer/move_base/TebLocalPlannerROS/dt_ref
rosparam get /tianracer/move_base/TebLocalPlannerROS/no_outer_iterations
rosparam get /tianracer/move_base/controller_frequency
rostopic hz /tianracer/cmd_vel
```

应为 false、0.15、3、15.0；实际控制频率需能达到目标。
若优化耗时使频率明显不足，使用 flow_tuning=false 对照，不继续加速。

## 记录运行质量

在车辆跑圈期间，另一个加载环境的终端：

```bash
rosrun tianracer_gazebo race_diagnostics.py __ns:=/tianracer _duration:=60 _output:=$HOME/ican_race_logs/flow_report.json
```

`topics.teb_resets.count` 是这次采集期间来自 move_base 的不可行重置次数。
它只统计订阅之后的日志；和整轮 50 次比较时要注意时间窗口不同。
如果仍有大量重置，提供 A 的完整日志和报告。需要进一步区分路径贴墙、局部定位误差或实际跟踪偏离，
不能仅从一条不可行警告断言是哪种原因。

## 验证和回退

本次共 77 项离线测试通过：此前 65 项、重复缓升回归 1 项、到点朝向/服务适配 8 项、局部配置 3 项。
验证航点坐标不变、帧检查与回退、发车前服务查询、行驶中无服务调用、碰撞几何与检查保留。
ROS 适配测试仍使用假消息，没有验证真实 Gazebo 行驶或圈速。
原有依赖真实 ROS 消息的 `test_waypoint_laps.py` 请在本机再运行：

```bash
cd ~/ican_race_ws/src/tianracer
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_waypoint_laps.py' -v
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_goal_heading.py' -v
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_flow_profile.py' -v
```

回退到此前能跑约 146.50 秒的配置：恢复 `smooth_race_demo.launch max_speed:=1.8`，
并将 C 的 align_goal_yaw 关闭。新到点朝向和 flow_tuning 在旧入口默认均关闭。
