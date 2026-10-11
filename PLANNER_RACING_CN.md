# 第三版不动问题的恢复修正

第三版在用户的真实 Gazebo 环境中未能正常行驶，已撤回其激进规划组合。
参数已加载、cmd_vel 达到 20 Hz，不能证明速度值非零或车辆能够跟踪路径。
用户补充日志确认首个目标以 state 4 中止：`Failed to find a valid plan. Even after executing recovery behaviors.`
这是导航规划失败，0/5 圈；后续消息断流发生在失败之后。当前仍不能定位具体哪一个参数使规划失效。

## 当前修正

- 恢复默认原链路的 `navfn/NavfnROS`，不再切换 GlobalPlanner。
- 代价地图、未知区域处理、膨胀半径和障碍代价恢复基础配置。
- TEB 的预瞄、参考点、迭代数、障碍/运动学权重等全部恢复基础配置。
- 控制频率回到 10 Hz，默认最大速度回到 1.8 m/s。
- 保留指令超时停车，关闭额外的输出平滑和曲率限速。
- 修复 watchdog 直接转换模式下零速消息暂时丢失转角的问题。

`planner_race_demo.launch` 现在是恢复入口，名称为兼容已有命令而保留。
它不再启用第三版 GlobalPlanner/高约束优化，也不需要录线。
`max_speed` 是唯一用于提速的参数；先恢复行驶，再逐档测试。
原版官方赛道、车型、裁判、地图和航点没有修改。

## 更新并完整重启

先在各终端 Ctrl+C 停止本次自己启动的跑圈、仿真和裁判。
不要仅在运行中设置 rosparam：旧插件实例和已加载代价地图需要重新初始化。

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source devel/setup.bash
rospack find tianracer_gazebo
```

包应来自 `~/ican_race_ws/src/tianracer/`。每个新终端都加载上述两个 setup。
已安装所需依赖时无需每次运行 rosdep update。
若 rosdep 未初始化，使用 `rosdep update --include-eol-distros`；仅在默认源列表文件不存在时先做一次 `sudo rosdep init`。

终端 A：

```bash
roslaunch tianracer_gazebo planner_race_demo.launch max_speed:=1.8
```

终端 B：

```bash
roslaunch tianracer_sim_referee referee.launch
```

车辆停稳、官方环境和绑定正常后点击准备发车。终端 C：

```bash
roslaunch tianracer_gazebo waypoint_laps.launch world:=tianracer_racetrack robot_name:=tianracer laps:=5 switch_distance:=0.5
```

启动后检查：

```bash
rosparam get /tianracer/move_base/base_global_planner
rosparam get /tianracer/move_base/TebLocalPlannerROS/global_plan_viapoint_sep
rosparam get /tianracer/move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist
rosparam get /tianracer/move_base/controller_frequency
```

应分别为 `navfn/NavfnROS`、-1、3.0、10.0。
如果仍为 GlobalPlanner、0.5、5.0 或 20，说明代码/进程没有完全更新或重启。

## 能行驶之后才提速

先用 1.8 完成一轮有效五圈，再分别测试 2.0、2.2、2.4：

```bash
roslaunch tianracer_gazebo planner_race_demo.launch max_speed:=2.0
```

每次更改都重启自己的仿真并准备新轮次。保持换点距离 0.5 和其他参数不变。
原基础转弯速度预算保留，因此提高直线速度上限不代表所有弯道都会同幅提速。
比较至少三轮的官方五圈总用时与完赛率；用户已有基准为 167.955 秒。
这次修正的首要目标是恢复行驶，不能据此宣称已经提速。

## 如果仍不动：采集实际指令

在 A/B/C 保持运行、航点目标仍活跃、Gazebo 没有暂停时，新终端加载环境后执行：

```bash
rosrun tianracer_gazebo race_diagnostics.py __ns:=/tianracer _duration:=10
```

这个脚本只读，不发送车辆指令；采集 10 个墙钟秒后退出。
报告保存到 `~/ican_race_logs/motion_report.json` 并打印到终端。
将报告和终端 A 最后 30 行 WARN/ERROR 一起发来。

- cmd_vel 有消息但 nonzero_count 为 0：规划端没有发出前进速度，需要检查目标状态和规划错误。
- cmd_vel 有非零速度但 Ackermann 没有：检查转换节点、命名空间和超时。
- Ackermann 有非零速度但 odom 近零：检查底盘控制器、急停、碰撞/卡住或物理执行。
- simulation_elapsed_s 接近 0：先检查 Gazebo 是否暂停。
- cmd_vel 断流：可能是导航失败、目标结束或恢复流程，不能仅凭频率确定原因。

必要时补充单条消息：

```bash
rostopic echo -n 1 /tianracer/cmd_vel
rostopic echo -n 1 /tianracer/ackermann_cmd_stamped
```

没有消息时单条 echo 会等待，所以优先使用有时间上限的诊断脚本。

## 验证范围

本次 30 项离线控制测试和 4 项配置测试通过。
新的配置检查确认基础 TEB 参数完整保留，没有附加代价地图覆盖；
控制测试检查零速转角保留和指令超时停车。
这不是 Gazebo 行驶验证。云端没有 ROS/Gazebo，仍需本机确认恢复行驶。
