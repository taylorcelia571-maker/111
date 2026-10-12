# 从 82 秒稳定基线继续提速

最新测试 lateral_accel=6.8 触发保护停车，已取消将它作为下一步提速建议；保留 6.5，按 [首圈和连续纠偏修正](SMOOTH_PURSUIT_CN.md) 测试控制变化。以下 6.8 命令为上一轮试验记录，当前不要继续使用。

用户最新反馈：旧 pursuit 模式有效五圈约 82 秒，无左右摆动和撞墙；新 curvature_feedback 模式会停止。后续性能比较先保持旧模式，不将新版停车原因猜成某一种故障，也不取消保护。

## 为什么 max_speed 从 8 到 9 没有收益

max_speed 是上限，不是强制速度。曲率、横向加速度、偏航角速度、转向速率和前后相邻路段的加减速包络共同限速；跟踪误差和激光保护还会在线进一步降速。正式判断仍使用里程计和裁判。

同一条 43.50 m 路径，按当前 max_speed=8、lateral_accel=6.5、acceleration=5、braking=3、yaw_rate=3.8 计算：

| 改动 | 模型五圈/秒 |
| --- | ---: |
| 当前预算 | 72.037 |
| 只把 max_speed 改到 9 | 72.037 |
| 只把 braking 改到 3.3 | 71.503 |
| braking=3.3，lateral_accel=6.8 | 70.275 |
| lateral_accel=8，braking=5，acceleration=6，yaw_rate=4.5 | 63.573 |
| lateral_accel=9，braking=6，acceleration=6，yaw_rate=5.5 | 60.064 |

这是固定路线的速度包络计算，**不含起步、在线保护额外减速、执行延迟、侧滑和定位误差，也不是车辆物理能力验证**。后两组不能当作稳定可用参数推荐：即使模型接近一分钟，也不代表实际五圈能在一分钟内完成。82 秒降到 60 秒需要平均圈时从 16.4 秒降到 12 秒，不能靠修改一个不生效的上限实现。

新 `planned_course_report.json` 加入 `peak_planned_speed`、`speed_ceiling_binding` 和 `same_route_parameter_sensitivity`。后者在同一路线分别改变一个预算，比较模型收益；只在静止规划阶段计算，不在行驶控制循环中增加优化任务。

## 一次只改变一个有收益的参数

先更新仓库并构建，所有终端加载实际运行的工作空间。沿用已完成 82 秒那一轮的仿真入口、轮速前馈设置和裁判流程，只替换控制终端命令。每次试验从默认静止起点开始，不能沿用停车位置直接起步。

稳定基线（保持旧 pursuit 模式，无新增关节反馈依赖）：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=8.0 lateral_accel:=6.5 acceleration:=5.0 braking:=3.0 yaw_rate:=3.8
```

首个提速试验仅将 lateral_accel 从 6.5 改到 6.8。这使同一曲率下的速度预算提高约 2.3%，没有同时提高加速、制动预算或改控制器：

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=8.0 lateral_accel:=6.8 acceleration:=5.0 braking:=3.0 yaw_rate:=3.8
```

这是待实测的试验，不是稳定性保证。保持响应时间预留 0.25 秒和所有停车保护。至少完成三次正式有效五圈，对比各圈用时和有无纠偏、撞墙、保护停车；出现上述问题就回到基线。未得到这一步稳定成绩前，不执行表中的高预算组合。

本次没有将 rapid 自动切换为新横向控制器，也没有调整官方车辆、地图、裁判或 PID。新版停止需要原始错误；不要用更高速度绕过它。

## 新版停车原因如何保留

更新后，即使反馈已过期，status 服务仍返回原始 `last_fault`，包含故障原因、仿真时间、最后控制状态和定位反馈。停车后在保持控制节点运行的情况下执行：

```bash
rosservice call /tianracer/continuous_race/status "{}"
```

传感器过期时 success=False 是预期诊断结果；message 中的 `last_fault.reason` 才是这次自动停车的原始原因。手动 stop 不清除故障记录，成功开始新轮才清除。节点关闭或重启后仍需查看原终端日志。
