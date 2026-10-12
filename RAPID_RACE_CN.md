# 提高直线与过弯速度，定位实际瓶颈

最新反馈旧模式稳定五圈约 82 秒，新横向控制器会停车；按 [82 秒基线提速说明](SPEED_TUNING_CN.md) 进行性能比较，并保留新版原始错误。不要仅提高 max_speed 或自动切换控制器。

此前高预算组合曾出现多次跟踪保护停车，并提供了 [完赛恢复步骤](FINISH_RECOVERY_CN.md) 中的 `finish_race.launch`。最新反馈该新控制器仍会停止，因此当前性能比较以用户已稳定完成的旧模式为基线。rapid 默认保留 0.25 秒响应时间，模型圈时不能代替实际成绩。

已验证的官方五圈基线是 92.463 秒（max_speed=4、lateral_accel=5）。本次增加联动调参入口、速度限制报告和弯道偏移候选。**尚未在 Gazebo 达到或验证五圈低于 60 秒。** 新配置的速度包络模型约为 76.2 秒，不含起步、侧滑与定位误差，不能当作正式成绩。

## 为什么只加 max_speed 几乎不变

在 43.50 米候选路线和原预算下，模型中约 60% 时间受加减速包络限制，约 35% 受横向加速度限制，仅约 5% 达到直线速度上限。原代码还有固定的 3.2 rad/s 航向变化率和 2 rad/s 转向变化率限制，只调 max_speed 不会改变它们。4 提到 6 的模型收益不足半秒/五圈。

现在规划器、路线评分与控制器使用同一组可配置限制，并在实时追踪中检查横向加速度和航向变化率。未修改轮胎、摩擦、车身、轮距、车辆模型、官方地图或裁判，也未关闭激光和定位保护。

## 新配置与基线

| 参数 | 已完赛基线 | rapid_race 默认值 | 作用 |
| --- | ---: | ---: | --- |
| max_speed | 4.0 | 5.0 | 直线速度上限，m/s |
| lateral_accel | 5.0 | 6.0 | 过弯预算，m/s²；同曲率允许速度约提高 9.5% |
| acceleration | 2.0 | 3.0 | 出弯/直线加速预算，m/s² |
| braking | 3.0 | 3.0 | 制动预算，保留原值 |
| yaw_rate | 3.2 | 4.0 | 航向变化率预算，rad/s |
| steering_rate | 2.0 | 2.0 | 转向变化率预算，rad/s，保留原值 |

这些是控制预算，未实测证实等于车辆物理能力。新入口自动启用快速起步和按预计时间选线。先比较一次只增加加速能力，再测试整组参数，避免无法判断某次失效来自哪里。

## 更新与启动

停止旧导航。仿真继续使用 planned_race_demo.launch，裁判继续使用 referee.launch，三个终端保持打开。每个新终端先加载两行环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
```

更新仓库：

```bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
ls "$(rospack find tianracer_gazebo)/launch/rapid_race.launch"
```

导航终端 C，第一轮保持已完赛弯道预算，先提高加速能力：

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=4.0 lateral_accel:=5.0 acceleration:=3.0 yaw_rate:=3.2
```

完成并停止该轮导航后，第二轮测试新整组预算：

```bash
roslaunch tianracer_gazebo rapid_race.launch
```

等待 Ready，查看 Speed budget report；规划在发车前进行，不算进首圈。裁判准备发车后，另一终端：

```bash
rosservice call /tianracer/continuous_race/status "{}"
rosservice call /tianracer/continuous_race/start "{}"
```

裁判锁定五圈后立即停止车辆：

```bash
rosservice call /tianracer/continuous_race/stop "{}"
```

不要同时运行 waypoint_laps、move_base 或另一套 continuous_race。仍必须以官方 5/5、模型校验、无碰撞和总用时判断有效收益。

## 报告与后续决策

发车前保存 `~/ican_race_logs/planned_course_report.json`，包括参数、预计圈时、达到 60 秒所需平均速度以及各限制占预计用时的比例。`model_five_seconds` 不含起步，不是比赛成绩。

行驶日志新增：

```text
Reference ... v=... measured=... error=... limit=...
```

v 是激光限制之后的最终指令速度，measured 是里程计实测速度。limit 分别说明加速、规划包络、起步、跟踪误差、实时转向曲率或激光限制。status 还返回最近的 planned_speed、live_curvature、command_speed、output_speed、measured_speed。若指令明显高于实测速率，才有证据继续检查轮速执行和 PID；不盲目补偿速度以掩盖侧滑。

60 秒要求这条路线的平均速度超过约 3.625 m/s，而紧弯与加减速会压低平均速度。当前预算的模型本身也超过 60 秒，所以不能称为“一分钟方案”。需要用新的官方完赛日志和报告校准可实现的横向/纵向能力，再决定下一步；不要把未经验证的强制动数值填进规划器来延后刹车。

## 路线选择的边界

新增小幅弯道内外偏移候选，每条候选均检查连续线段距离、转向曲率和原航点顺序。评分用完整周期速度包络的预计时间，不用弯道数量替代圈时。受赛道墙体和检测线约束，无法自由改成弯道更少的任意路线。这是候选集中的较快路线，不是证明全局最优。当前地图和预算下偏移候选未优于 43.50 米路线，程序因此保留原候选；不为了展示“换路线”而换成更慢路径。

离线验证覆盖新参数传递、约束一致性、限制报告及带假设 80ms 执行器响应的五圈跟踪。没有 ROS/Gazebo 实车身动态验证，不能从这些测试推出不翻车或 60 秒完赛。
