# 已启动后立即停车：起点接入阶段的加速回归

本次 start 服务成功，首次实测运动发生在 0.047 秒；约 0.85 秒时实测速度达到 4.144 m/s，纠偏指令已降到 1.082 m/s；约 0.95 秒时参考线偏差达到 0.1757 m，超过 0.150 m 阈值，控制器主动停车。最后 TF 年龄 0.005 秒，日志未显示反馈过期。不能将 start 服务受理成功当作后续能够持续运行。

上一轮连接线绕开了 departure 限速，直接使用 acceleration=5。这个启动加速回归使车辆过早进入高速度，跟踪保护随即停止。新的修复只改变 smooth_pursuit 连接线的起步阶段：

* 接入阶段默认速度上限 1.2 m/s、加速上限 1.5 m/s²。
* smooth_pursuit 只有起点方向差不超过约 2.9° 才选择连接线；方向偏差较大时回到低速对齐。
* 完成连接线后，还需位置偏差小于 2.5 cm、航向误差小于约 2.9°、转向跟踪误差小于约 2.9°，连续满足 0.3 秒，才解除接入限速。
* 前 5 m 保持较低加速预算，避免刚完成接入就猛烈加速；之后恢复传入的正常预算。此距离不在每圈重新计算。
* 每次重新 start 清除上一轮的就绪计数，不能靠重复调用 start 跳过检查。
* 15 cm 偏差保护、墙距与激光保护保持不变；没有增大阈值来让车辆越界继续走。故障记录补充里程计横向速度和偏航角速度，以区分后续实际侧滑与定位变化。

## 先重置这一轮

车辆已停在途中且偏差超过起步阈值，不要在原地反复 start。先结束裁判当前轮，停止控制器与旧仿真，重新启动原先的官方场景和裁判，让车辆回到默认静止起点，并重新确认官方环境。

更新：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source devel/setup.bash
```

每个终端都加载 Noetic 和 ican_race_ws 环境。控制终端使用新的较保守起步配置，等待 Ready：

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=8.0 lateral_accel:=6.5 acceleration:=5.0 braking:=3.0 yaw_rate:=3.8 lateral_control:=smooth_pursuit start_blend:=true response_delay:=0.25 startup_speed:=1.2 startup_acceleration:=1.5 predict_pose:=false refine_line:=false
```

确认启动参数中有 startup_speed=1.2、startup_acceleration=1.5。另一个终端先调用静止 prepare：

```bash
rosservice call /tianracer/continuous_race/prepare "{}"
```

成功后确认车辆静止，裁判点击“准备发车”，再调用：

```bash
rosservice call /tianracer/continuous_race/start "{}"
```

状态中的 launch_limited=True 表示起步保护仍在生效，startup_connector 表示接入限速。launch_ready_seconds 连续到 0.3 才能释放限速；active_acceleration 在前 5 m 不超过 1.5。run_report 保留本次限制原因和故障。

## 回到上一轮已完成五圈的配置

若要先恢复你已完成 85.225 秒的配置，重置场景后使用以下命令，关闭连接线并恢复原来的响应时间预留：

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=8.0 lateral_accel:=6.5 acceleration:=5.0 braking:=3.0 yaw_rate:=3.8 lateral_control:=smooth_pursuit start_blend:=false response_delay:=0.25 predict_pose:=false refine_line:=false
```

该回退会恢复低速对齐，可能仍有较长首圈，但它是用户已完成五圈的配置；不要从已经偏离的停车位置直接使用它重启。

## 验证范围

新增测试使用日志的初始位置、方向和 8/6.5/5 高预算，验证前一秒不能再请求 4 m/s 级起步；验证就绪状态重置，以及真实地图上多种假设执行延迟的连续五圈跟踪。它们不包含真实 Gazebo 轮胎、侧滑或 AMCL 收敛行为，不能证明本地已恢复完赛。云端未运行实际 Gazebo；本次没有声明一分钟目标已实现。
