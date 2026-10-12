# 从约 142 秒向两分钟目标推进：减负与原生曲率保持

用户报告最新五圈约 2 分 22 秒。本次上传日志中，轨迹不可行重置降到 4 次，
此前为 50 次；没有看到控制循环超时警告。因此“所有卡顿都是计算慢”还不能确认。
优先保留已改善的 flow 配置、到点朝向预规划和轮速前馈，避免再次回退成大量重置。

## 新增改动

- 局部代价地图 15×15 m → 8×8 m，0.05 m 分辨率、15 Hz 更新、障碍层和碰撞检查不变。
  网格从 90,000 到 25,600 个，减少约 71.6%。半宽 4 m 仍覆盖 3 m 预瞄及传感器/车体余量。
- 代价地图发布到 RViz 的频率 10 → 2 Hz，真实感知和规划更新频率没有降低。
- 全局规划在新目标和恢复时执行，不每两秒重新计算相同目标；TEB 和现场障碍仍实时更新。
  本模式针对固定赛道。动态障碍需改变全局路线时，必须验证恢复/重新规划行为。
- TEB 原生 `use_proportional_saturation: true`：触及线速度/角速度预算时同时缩放，
  尽量保留 `omega / v`，而不是独立截断角速度导致转弯半径变大。
- `max_vel_trans` 同步设置为 max_speed，避免另一个平移上限掩盖调速效果。
- 新入口默认关闭 RViz 渲染，Gazebo 界面和官方裁判保留。调试时可 `use_rviz:=true`。
- 保留上一版的 TEB 时间步长、迭代、避障权重、模型足迹、最小转弯半径和可行性检查。

没有修改官方模型、物理参数、摩擦、地图、航点位置、裁判或 PID。
这不是保证 120 秒以内的结果；需要本机验证更高速度下的跟踪和有效完赛率。

## 更新与运行

先停止自己运行的 A/B/C。每个新终端都 source Noetic 和 `~/ican_race_ws/devel/setup.bash`。

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source devel/setup.bash
rospack find tianracer_gazebo
```

终端 A，先做 2.0 m/s 档：

```bash
roslaunch tianracer_gazebo fast_race_demo.launch max_speed:=2.0 turn_rate_budget:=3.6 use_rviz:=false
```

终端 B：

```bash
roslaunch tianracer_sim_referee referee.launch
```

绑定和官方环境有效、车在起终点附近停稳后准备发车。终端 C 保留朝向预规划：

```bash
roslaunch tianracer_gazebo waypoint_laps.launch world:=tianracer_racetrack robot_name:=tianracer laps:=5 switch_distance:=0.5 align_goal_yaw:=true
```

不需要录线。C 在发车前生成四段参考到点朝向，跑圈中不调用 make_plan 服务。
不要同时开启旧速度调度、低通、path_bias 或其他控制器。
RViz 关闭后没有二维路径窗口是正常的，不影响 Gazebo 或裁判检测；调试时开启 use_rviz 即可。

## 分档测试，不把计算值当安全极限

保留原来的 142 秒完赛成绩作为基准。每档至少三次有效五圈：

1. 2.0 m/s，角速度预算保持 3.6。
2. 稳定后依次试 2.1、2.2。此次原生同比例限速与之前直接把全程上限调到 2.2 不同，
   但仍不保证此前撞墙点已完全解决；尤其检查第二次回到起点附近。
3. 只有更低档有稳定完赛证据后，才试 2.3、2.4。任一档擦墙、漏线或重置明显增加，都回到上一档。
4. 先不要提高 turn_rate_budget。若日志证明弯道被预算限制而非跟踪误差，
   可独立比较 3.8，但不能和直线速度同时提高；提高该预算会允许紧弯更快，风险也增加。

0.5 m 半径在 3.6 rad/s 预算下对应约 1.8 m/s，0.75 m 半径对应 2.7 m/s。
原生同比例限速在优化轨迹超过预算时尽量保持曲率；不是所有弯道都强制使用 1.8。
控制器仍可因避障、加速度和优化结果给出更低速度，不能强行覆盖其零速。

## “最大不翻车速度”的计算边界

新增只读计算器：

```bash
rosrun tianracer_gazebo speed_budget.py
```

它读取原始模型的质量、惯性原点和关节变换。该源码名义静止状态：
总质量约 13.25 kg，重心高度约 33.6 mm，轮距按 0.18 m 计算。
这些是模型文件的静态值，不是运行中的重心测量。

理想平地刚体翻滚阈值：`v_roll = sqrt(g * (track/2 - abs(y_com)) * R / h_com)`。
在 R=0.5 m 时约 3.63 m/s。**这只是理想静态翻滚模型，绝不能把 3.63 当作可用转弯速度。**
抓地、侧滑、控制误差、路面接触和撞墙会先限制速度。

摩擦场景公式：`v_grip = sqrt(mu_eff * g * R)`。R=0.5 m 时：

| 假设有效摩擦系数 | 理论抓地速度 |
| --- | --- |
| 0.6 | 1.72 m/s |
| 0.8 | 1.98 m/s |
| 1.0 | 2.21 m/s |

mu_eff 不是已测得数值，不能直接使用轮子材料中单个 mu 参数替代轮胎/赛道有效接触。
这些场景说明为什么“还没翻车”并不等于能高速准确过弯。

计算器还给出假设感知 2.5 m、减速 1.8 m/s²、反应 0.25 s、余量 0.3 m 时的制动预算约 2.4 m/s。
减速能力是规划假设，不是实测；这也不是 2.4 m/s 的安全认证。

142 → 120 秒需要约 15.5% 时间下降；如果全部时间随速度反比变化，1.8 档的等效速度需约 2.13。
实际存在紧弯、换点和停顿，因此只设置 2.13 不保证 120 秒，不能预先承诺必达目标。

## 确认配置与衡量“思考时间”

```bash
rosparam get /tianracer/move_base/local_costmap/width
rosparam get /tianracer/move_base/local_costmap/resolution
rosparam get /tianracer/move_base/planner_frequency
rosparam get /tianracer/move_base/TebLocalPlannerROS/use_proportional_saturation
rosparam get /tianracer/move_base/TebLocalPlannerROS/max_vel_trans
```

应为 8.0、0.05、0.0、true、当前 max_speed。
该参数已依据官方 Noetic TEB 源码确认支持。如果本机插件明显更旧，
可用 `rosrun dynamic_reconfigure dynparam get /tianracer/move_base/TebLocalPlannerROS` 确認原生配置支持该字段；
参数服务器存有某个名称本身不能证明旧插件使用了它。

跑圈时另开终端采集：

```bash
rosrun tianracer_gazebo race_diagnostics.py __ns:=/tianracer _duration:=60 _output:=$HOME/ican_race_logs/fast_report.json
```

新增统计：`control_deadlines`、`map_deadlines`、指令间隔 p95/max、超过 0.2 s 的指令间隙数。
在车辆正常行驶期间采集；无目标等待和结束阶段也会造成消息间隙，不能误判为计算超时。
结合 TEB 重置次数、实际欠速、转向延迟和官方五圈用时，再决定是否值得提高速度。

## 验证与回退

本次 85 项离线测试通过，涵盖保持曲率的限速算例、计算预算、重心变换、网格覆盖、参数传递和此前控制回归。
这些是数学/配置/假消息测试，未实测你的 Gazebo 车辆、有效摩擦或官方两分钟成绩。

原版足迹、模型、地图和所有碰撞检查保持不变。新性能模式在旧入口默认关闭。
回退：重新启动此前 `flow_race_demo.launch max_speed:=1.8 flow_tuning:=true`，C 保持 align_goal_yaw=true。
这样保留已经被本机验证约 142 秒的配置，不覆盖现有最好成绩。

参考原生限速逻辑：
https://github.com/rst-tu-dortmund/teb_local_planner/blob/noetic-devel/src/teb_local_planner_ros.cpp
