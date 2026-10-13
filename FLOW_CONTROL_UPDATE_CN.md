# 启动、过弯和转向卡顿：控制逻辑优化

## 已完成的修改

1. smooth_pursuit 可选择 startup_release_early=true，使用与race_feedback相同的严格就绪检查：行进>=0.6m、控制跟踪误差<0.025m、航向误差<0.05rad、目标/实测转角差<0.05rad并持续0.3s。未就绪仍保持启动速度与加速度限制。旧默认false保留，旧5m加速预留不再阻挡已按新条件就绪的可选模式。起点连接线地图检查仍保留。
2. smooth_pursuit 曲率前馈的当前曲率改用实际投影处的线性插值，与前方曲率一致，降低索引切换产生的转向跳变。没有提高转角、转角变化率或改车辆PID。
3. 新增 speed_recovery_hold：风险诱发速度下降后短暂禁止重新加速，避免立即加速后再次降速。只压低输出，不抬高任何限速，不延迟制动或停车。倒计时有限，不因持续同一限速永久保持；start清空上轮状态。默认0关闭，可从0.04s小步试。
4. 控制状态增加 upstream_reason、recovery_hold_remaining、speed_recovery_hold；最终输出和雷达限制前速度必须区分。分类有重叠，不能相加推断节省时间。
5. planned_race公共入口旧默认route_margin=.26、response_delay=0现在加强为.34/.25。race_tuning/demo原来已经显式使用.34/.25，本次继续保留。墙距停车.23、偏差停车.15、雷达/数据新鲜度/关节检查不变。官方车辆、地图、裁判不改。

## 验证范围

88项针对性回归通过：运行适配52、smooth pursuit14、time_attack9、投影一致性3、转向预瞄3、新恢复与启动7。包含激光硬停车不被等待延迟、手动stop零速、重启清空恢复状态、输出不超过上游限制、就绪不足不释放、曲率采样边界连续性。

Python3.8语法与XML验证通过。真实ROS Noetic用roslaunch --dump-params解析了startup_release_early=true、speed_recovery_hold=.08、steering_preview_time=.12，确认控制器和保护参数转发。

假设150ms一阶响应的自行车模型：修改后旧启动方式首圈19.08s；启用提前释放首圈18.54s，模型五圈88.48 -> 87.94s。增加.08s恢复等待后模型首圈18.58s、五圈88.18s。250ms模型中等待.08降低峰值偏差，但总时间可能增加。所以默认等待仍关闭，不把它当通用提速参数。

所有这些是模型/离线检查；没有Gazebo五圈实测，也未验证用户首弯故障消失、无侧滑/翻车/碰撞、官方小于60秒。当前用户报告最佳约82秒不变。本次未分析超32MiB压缩包，不宣称已读bag证明根因。

## 更新并试第一组

先stop并结束失败轮，停止旧控制器、场景、裁判。更新失败不能继续启动：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git switch main && git pull --ff-only && cd ~/ican_race_ws && catkin_make
source ~/ican_race_ws/devel/setup.bash
```

如有本地修改阻止切换/拉取，保留报错，不使用reset强制覆盖。每个新终端加载两个setup。

第一组保持所有预算，且只启用提前启动释放：

```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true run_report:=$HOME/ican_race_logs/flow_start.json
```

该命令同时启动官方场景/裁判/控制器，不另启旧节点。等待Ready，裁判检测、绑定并准备发车后只start一次：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

## 参数怎么改

所有参数启动时读取，改完需要停止旧控制器并从官方默认静止起点重启；运行中rosparam set不保证重新构建控制器或规划。

* startup_release_early：false/true。先单独验证true。条件不足时不能强行解除；若首弯更差恢复false。
* speed_recovery_hold：默认0，合法0–0.4s。先0.04，再0.08；越长不一定越快。风险停车仍必须结束本轮，不能靠等待重启。
* steering_preview_time：默认0.10，先0.12。过早切弯、左右摆动或时间变慢就恢复.10。不要同轮同时改反馈增益。
* lateral_accel/acceleration/braking/yaw_rate本轮保留6.5/6.6/3.6/3.8；steering_feedback_gain保留.25。max_speed=10不是主要提速参数。

第二组，只在第一组安全有效后，增加一个变量：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true speed_recovery_hold:=0.04 run_report:=$HOME/ican_race_logs/flow_hold04.json
```

转向预瞄独立对照第一组，不从带hold组直接跳转来做归因：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true steering_preview_time:=0.12 run_report:=$HOME/ican_race_logs/flow_steer12.json
```

每次只动一个变量，不要求连续三次。官方有效5/5、last_fault=null、无翻车/碰撞/保护停车且准确官方总时间小于当前最佳才保留。只改善模型、启动或未完成圈数不能当成绩。<60秒必须官方original time<60000000微秒及有效结果。

## 失败与回退

风险限速、明显超目标速度、偏差接近15cm、翻车/碰撞或控制停车立即：
```bash
rosservice call /tianracer/continuous_race/stop '{}'
```
不继续start。结束本轮、重置场景。参数回退（保留本次连续插值代码修复）：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=false speed_recovery_hold:=0 steering_preview_time:=0.10 run_report:=$HOME/ican_race_logs/flow_fallback.json
```

若需代码也回到本版前，工作树干净且全部节点停止时，可git switch --detach adebf3b9dc40483b31df00bf4a1d6c7a810cc1b9，重编译再使用此前入口/参数。该操作会离开main；返回新版须git switch main再更新。

回退组不是新的82秒官方证明；要恢复Run2需确认原控制方式及7.0/7.0/3.8/4.0参数与该成绩确实对应。保留成绩依据，不改控制器报告official_result字段。
