> 最新弯道偏离修正：日志确认在原始 TF 反馈下，出弯车姿与路径切线不一致，误差增至 0.1599m，触发原有 0.15m 停车阈值。新增控制在小误差且航向朝外偏离时提前限制指令速度（纠偏阶段最多 1m/s）、缩短预瞄距离，并随回到路线恢复原速度包络；正常贴合路线不启用该限制。日志 limit=outward_error_recovery，status 包含 heading_error/outward_heading/recovery_speed_cap。0.15m 阈值和其他保护不变，已经越界的停车车姿不能直接重启发车，应重新建立起点场景。新增扰动测试覆盖假设 120ms 执行器响应，仍待 Gazebo 完整五圈验证。

> 激光缓存停车修正：旧版缓存超过 100ms 时直接停车，未尝试重新取得新扫描 TF。现保留最近 16 帧，优先使用 100ms 内、采集时刻变换已验证的最新扫描；缓存过期后对当前新鲜扫描最多等待 50ms 的 TF，成功后刷新车姿与控制时间再运算。新鲜度检查仍保留，不能验证变换或真实数据过期仍停车。当前先维持 77 秒配置及三个新模块关闭，待完整五圈确认恢复后再单独调参。

> 起步失败后的修正：已观察到同时启用 start_blend 和 predict_pose 时，在起步一秒内发生 0.1599m 跟踪偏差并停车。报错 pose 是预测后车姿，并非原始 TF；尚不能确定单项根因。日志和 status 现在同时返回原始/控制车姿与 prediction_applied。rapid_race 和 sprint_trial 现默认关闭这两项及 refine_line，回到此前完赛的配置组合；本文原先的默认启用描述和理想模型时间不能用来证明当前默认配置达标。先验证原参数五圈，再仅开启 refine_line 比较，暂不提高速度或制动预算。

# 起步接入、逐弯优化和一分钟目标试验

最近的官方成绩为 77 秒，启动参数为 max_speed=8、lateral_accel=6.5、acceleration=5、yaw_rate=3.8。本次修改默认不改变 rapid_race 的速度预算；先用同样参数核对算法收益。**一分钟官方完赛尚未验证，不能保证用下面的试验参数达到。**

## 实际修改

1. rapid_race 现在启用 `start_blend`：车姿贴近路径、航向相近、前方足够直且整段地图距离合格时，生成一个从当前后轴位置平滑接入路线的 2.5 米过渡段。它不等待固定低速对齐，不提前移动车辆或修改起点。过渡位置与导数连续，后续每圈不重复此过程。条件失败则回退原对齐控制，保留定位误差与激光保护。
2. `refine_line` 对各弯道分别做局部偏移搜索，最多两轮，按预计圈时接受改动。原来统一偏移所有弯道的方法没有找到更好候选，这次逐弯搜索找到了 15 次有效改动。用最近的实际预算，后轴路线约从 43.50 米变为 43.05 米，速度包络模型每圈约减少 0.43 秒。仍保留完整线段障碍距离下限、转向和引导航点顺序检查。
3. `predict_pose` 使用时间戳匹配的里程计对不超过 80ms 的反馈延迟做常速度/航向变化率预测，包含横向速度；不从过期数据做长时间外推，不提高 300ms 时效上限。官方里程计发布周期为 50ms，直接用旧车姿会增加高速转弯的控制延迟。
4. 连续控制不再每次阻塞等待最新扫描的 TF；可暂用已经验证采集时刻 TF、且不超过 100ms 的激光缓存，过期则停车。该修正减少扫描与里程计不同发布频率引起的控制阻塞。
5. 新 RViz 显示当前后轴路径（绿色 `/tianracer/race_path`）和优化前候选（橙色 `/tianracer/race_baseline`）。旧 Global Plan 订阅 TEB 话题，不能用来判断连续控制器是否换线。修改的是同一赛道走廊内的形状，不是跳过赛道必经区域。

## 更新后先测试原预算

停止旧导航，更新并加载环境：

```bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
```

保持官方仿真和裁判运行，导航终端：

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=8.0 lateral_accel:=6.5 acceleration:=5.0 yaw_rate:=3.8
```

如果重启场景，仍用 planned_race_demo.launch，它会自动载入新 RViz 配置。若不重启场景，在现有 RViz 添加两个 Path 并设置上述话题，也能对比。

等待 Ready 和 Time-based route selection。后者包含 route_changed、refinement_moves、refinement_gain_seconds、route_sha256（中心线路径四位小数指纹）。这组地图和参数下应该 route_changed=True；若 False，程序确实保留了候选，不应声称换了线。

裁判准备后调用 start：

```bash
rosservice call /tianracer/continuous_race/start "{}"
```

`Race start mode` 应显示 connector 或回退 alignment 的原因；`Startup: measured motion after ...` 是实际运动等待时间。`First reference loop` 仅用于控制诊断，不是官方圈时。status 返回 startup 数据。控制器不会提前为缩短计时而移动车辆。

## 一分钟目标试验预算

新增 `sprint_trial.launch`，是未在 Gazebo 验证的能力预算试验，不是保证一分钟的入口。对比当前预算，增加的关键项是弯道与制动能力，并非再次增加 max_speed：

| 参数 | 最近已跑预算 | sprint_trial 默认预算 |
| --- | ---: | ---: |
| max_speed | 8 | 8 |
| lateral_accel | 6.5 | 9 |
| acceleration | 5 | 6 |
| braking | 3 | 6 |
| yaw_rate | 3.8 | 5.5 |
| steering_rate | 2 | 2 |

先用中间预算验证制动与过弯，另起一轮：

```bash
roslaunch tianracer_gazebo sprint_trial.launch lateral_accel:=8.0 acceleration:=5.0 braking:=4.0 yaw_rate:=4.5
```

只有车辆确实能稳定完成中间预算、实际速度与指令相符，才测试默认目标预算：

```bash
roslaunch tianracer_gazebo sprint_trial.launch
```

每轮停止上一套导航，等待 Ready、裁判准备后发车。不要同时运行多个控制器。braking=6 表示规划器假定车辆可以实现 6m/s² 制动，若实际达不到会刹车偏晚；必须通过实测验证，不能只看预计圈时。更高横向预算也需要验证实际抓地与稳定性，现有停车保护不构成“不撞墙不翻车”的保证。

目标预算下，本地速度包络模型约 58.23 秒（不含起步）。在假设 40ms 反馈延迟、80ms 执行器响应且没有轮胎侧滑的自行车模型中，开启预测后五个参考闭环约 59.94 秒，地图距离和跟踪误差合格；没有预测的同一模型因跟踪偏离停车。这只说明修正反馈延迟有意义，不能作为官方 60 秒成绩。ROS/Gazebo 车身动力学、真实反馈延迟和检测线判圈尚未验证。

## 验收与必要数据

必须看裁判 5/5 的总用时，保留每圈成绩、实际启动命令、导航日志和 `~/ican_race_logs/planned_course_report.json`。该报告的模型时间仍不包括起步，不能代替比赛计时。目标预算若仍高于 60 秒或保护触发，需要据这些数据定位真实限制；不会修改判圈、结果文件、官方地图、摩擦或车辆模型来满足目标。官方锁定五圈后照常调用 stop 制动。
