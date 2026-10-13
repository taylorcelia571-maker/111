# 本轮日志与受限转向反馈补偿试验

## 实测日志确定的瓶颈

本轮 rapid_race 使用 10/6.5/5.5/3/3.8 预算、smooth_pursuit、route_margin=0.34、start_blend=false、response_delay=0.25。日志未出现停车 ERROR，完成六个导航参考闭环；这不等于已提供官方五圈成绩。

start 后 0.094 秒就出现实测运动，首个参考闭环 22.494 秒，六闭环总用时 112.358 秒，后五个参考闭环平均约 17.973 秒。起步慢主要是前约 4–6 秒低速对线；不能把整个首圈额外时间都当作 ROS 启动等待。运行中多次 steering_slew 限速，目标与实际转角相差约 0.15–0.32 rad，个别指令速度降到 0.35 m/s、实际仍约 1.55 m/s。同步 bag 才能区分实际执行滞后、定位与侧滑的影响。

45.07 m 路线达到五圈小于 60 秒，要求含起步平均速度超过 3.755 m/s。原包络模型五圈 77.34 秒，含响应预留的代理约 87.24 秒；max_speed=10 不起约束，规划峰值约 6.04。模型不是物理极限证明，但现有预算与控制数据没有支持一分钟目标。此前修改没有完成这一目标。

## 本次可选算法

新增 steering_feedback_gain，默认 0，原命令仍保留原控制策略。试验档取 0.25，根据公开关节实际转角做软件补偿：

```text
执行器目标角 = 路径跟踪目标角 + gain × (路径跟踪目标角 − 实际转角)
```

目标和输出均受原 0.48 rad 转角、2 rad/s 指令变化率约束。弯道速度约束同时考虑跟踪目标、补偿目标、输出和实际转角中最大的曲率，不因补偿而放宽横向/偏航限制。启用补偿需新鲜关节反馈；40–80 ms 内逐渐减小额外补偿，超过 80 ms 回到原目标，超过原 300 ms 时限仍保护停车。保留墙距、激光、最大偏差、速度响应预留和预转向就绪检查。

这是外层软件补偿，没有修改原版车辆、轮胎、底层 PID、官方地图或裁判。增益限制 0–0.5，但本轮仅建议将 0.25 作为独立试验，不推荐直接调到最大值，也没有新认证的极限过弯速度。异常传感器或碰撞风险下仍保留保护停车，不能保证永不紧急制动。

## 离线结果，不是官方成绩

真实仓库地图、同样预算、0.34 m 净空、五个模型参考闭环、假设一阶执行器响应：

| 假设响应 | 补偿关闭 | gain=0.25 | gain=0.25 且逐弯细调 |
| --- | ---: | ---: | ---: |
| 80 ms | 89.08 s | 88.92 s | 87.28 s |
| 120 ms | 91.20 s | 89.78 s | 88.00 s |
| 200 ms | 108.46 s | 98.54 s | 96.12 s |

200 ms 假设下，首圈从约 23.22 降至 21.26 秒；80 ms 假设下首圈基本无变化。这说明收益依赖执行器响应，不是普遍提速保证。所有上述 0.34 m 候选的模型车辆中心最低净空至少约 0.317 m，偏差小于 0.15 m。另验证 40/80 ms 关节反馈延迟，五个模型闭环及原墙距/偏差约束通过。模型不含实际轮胎、AMCL、完整 Gazebo 动力学和裁判检测线；它没有达到 60 秒。

机器可读报告见 [STEERING_FEEDBACK_OFFLINE_RESULTS.json](STEERING_FEEDBACK_OFFLINE_RESULTS.json)。不能把这些时间上传或填写成官方成绩。

## 更新和本机验证

先停车并结束上一轮，退出旧控制、场景及裁判。保留本地修改；每行分别执行：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

每个新终端均先加载 Noetic 和工作空间。A 启动 `roslaunch tianracer_gazebo planned_race_demo.launch`，B 启动 `roslaunch tianracer_sim_referee referee.launch`。恢复默认静止起点，确认官方环境和正式测试绑定有效。C 只增加补偿，不同时改变速度预算、响应预留、起点连接线或路线细调：

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=10.0 lateral_accel:=6.5 acceleration:=5.5 braking:=3.0 yaw_rate:=3.8 lateral_control:=smooth_pursuit route_margin:=0.34 start_blend:=false response_delay:=0.25 predict_pose:=false refine_line:=false steering_feedback_gain:=0.25
```

等待 Ready 后，D：

```bash
rosservice call /tianracer/continuous_race/prepare '{}'
rosservice call /tianracer/continuous_race/status '{}'
```

等 steering_ready=true 且车体静止，再在裁判点击“准备发车”，然后：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

运行报告中 steering_feedback_gain 应为 0.25；正常新鲜反馈时 effective_steering_feedback_gain 为 0.25。若长期为 0，请核查 steering_feedback_age 和 joint_states 更新，不能据此继续加大增益。启动后仍需观察 ERROR；success=True 只表示接受发车。

官方有效五圈锁定后立即停止：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
```

按 [完整验证流程](RACE60_CURRENT_VALIDATION_CN.md) 保存同一轮控制 JSON、规划 JSON、官方各圈时间和同步 bag。比较 gain=0 与 0.25 各三轮：有效完赛率、首圈、中位数、转向失配和急减速次数。出现摆动、偏差增加或保护停车时回退 gain=0 并重建默认起点，不取消保护。

补偿档连续三轮稳定且实际更快后，可以再仅把 refine_line 改为 true，比较有墙距余量的逐弯优化；不能直接缩小 route_margin 或 response_delay 来凑模型时间。即使两项都有效，当前没有证实能达到一分钟。

新加的 `planned_race_demo.launch use_rviz:=false gui:=false` 允许在不改变车辆/物理参数的情况下关闭场景图形窗口，供计算资源有限或无显示的环境使用；它可能减少计算争用，不等于必然降低仿真计时。正式裁判仍需可用桌面。桌面默认设置保持原样。

## 云端实际验证状态

已准备 Ubuntu 20.04/ROS Noetic/Python 3.8.10 基础容器，但 Gazebo ROS 依赖未安装：官方 `snapshots.ros.org` 软件源请求返回 403，当前网络未放行该域名。没有绕过代理、关闭 TLS 或包签名校验。已保存仅添加此域名的云环境网络草稿；用户需在环境设置审核保存并发布后，才能继续安装与真实 Gazebo 验证。草稿保存不会立即改变当前网络。没有宣称完成实际 ROS/Gazebo 五圈验证。
