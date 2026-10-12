# 85.225 秒实测后的起步与速度修正

本轮官方五圈为 85.225 秒，首圈 20.961 秒，后四圈 16.050、16.084、16.093、16.037 秒。控制日志确认上一版 smooth_pursuit 已生效。首次实测运动仅用了 0.071 秒；随后约 5～6 秒仍处于 departure 低速对齐，偏差从约 7 cm 增至 12 cm。此前的起步修正没有达到提速目的。

## 本次修正

启用 smooth_pursuit 的起点连接线：从实际静止起点出发，用 2.5 m 的五次曲线平滑接到既有赛线。控制器同时使用连接线的位置、航向和曲率变化；不再将连接线的起点偏移当作跟踪误差急纠偏。保持原赛线的 15 cm 偏差停车保护、地图墙距和激光检查。

连接线只在起点偏差不超过 10 cm、方向差不超过约 8.6°、前方路线较直时生成；开始控制前检查连接线车辆中心的地图净空不小于 0.26 m。未通过检查会回到低速对齐，并在 Race start mode 中记录原因。不会因为选择连接线而忽略错误起点。

对 0.15 秒响应时间预留进行候选测试，以减少 0.25 秒预留对速度包络的额外提前限速。它不是已实测确定的舵机或轮速延迟，不能保证在本地保持稳定；如果实际滞后较大，应恢复 0.25 秒。

控制器自动生成 `~/ican_race_logs/control_run_report.json`，包含是否 prepare、实际起步模式、对齐耗时、参考圈时、每种限速原因累计时间，以及直线/弯道区域的平均实测与指令速度。报告使用仿真时钟，仅用于控制分析，不能当作官方成绩。每个参考圈和停止时写入；准备阶段不计入行驶指标。写盘错误不会阻止零速度停车指令。

## 更新与试验命令

先结束当前比赛轮并停止旧导航、仿真，从默认静止起点重新启动。更新代码：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source devel/setup.bash
```

沿用原来已跑通的官方场景和裁判入口。控制终端使用下面的候选配置，保持打开并等待 Ready：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=8.0 lateral_accel:=6.5 acceleration:=5.0 braking:=3.0 yaw_rate:=3.8 lateral_control:=smooth_pursuit start_blend:=true response_delay:=0.15 predict_pose:=false refine_line:=false
```

另一个加载相同环境的终端执行静止预转向：

```bash
rosservice call /tianracer/continuous_race/prepare "{}"
```

success=True 后，确认车辆静止，在裁判点击“准备发车”，再执行：

```bash
rosservice call /tianracer/continuous_race/start "{}"
```

应看到 `Race start mode: {"mode": "connector", ...}`。若仍为 alignment，请查看 reason；连接线路检查未通过时不可强行解除保护。确认没有打开 predict_pose，也没有提高 6.5 的弯道预算。完成官方五圈后停止控制器：

```bash
rosservice call /tianracer/continuous_race/stop "{}"
cat ~/ican_race_logs/control_run_report.json
```

若出现保护停车，先结束本轮，保留 ERROR 和报告。后续试验先将 response_delay 恢复到 0.25，保留其他参数，以区分提前减速预留和连接线的影响。需要全部回退则再将 start_blend 改成 false，即恢复上一轮配置。

## 一分钟目标与检查结果

单独消除首圈额外约 4.9 秒，总时间仍约为 80.3 秒。现有 6.5 弯道预算的固定路线速度包络模型五圈约 72 秒，尚未包含起步和在线限速；因此本次起步修正不能等同于已实现一分钟目标。当前也没有实测证据证明可稳定使用更高过弯预算，6.8 曾触发保护停车。

实际地图、假设 80/120/200 ms 一阶执行延迟并采用转角反馈的五圈自行车模型完成了候选检查。0.25→0.15 秒预留在这些模型中缩短总时间约 1.9～3.1 秒；开启连接线也降低了模型首圈对齐损失。这些是模型结果，不能当作本地 Gazebo 成绩，不能证明不会侧滑、翻车或停车。

云环境没有 ROS/Gazebo，本次没有真实五圈实测，**一分钟目标尚未达到或验证**。后续必须根据运行报告中直线与弯道实测速率、各限速原因耗时及执行滞后决定可提高的预算；不再单纯将 max_speed 从 8 调到 9，也不将未经验证的高预算当作稳定上限。
