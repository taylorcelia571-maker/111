# 路线与弯道提速版：最快模型候选与单变量实测

已知：用户确认上一版能完赛，但仍过慢。本版不处理上传包，不要求继续找数据文件。当前没有ROS2GO连接或Gazebo车辆依赖，因此以下时间全部是模型，不能称为官方实测成绩或证明60秒达标。约82秒仍是用户报告的当前官方最佳，准确比较要用裁判原始微秒值。

## 本版改动

- line_search_level=0保留原路线。1/2增加弯道进入/弯心/出弯的多尺度局部偏移搜索。每个候选重新验证整条路线净空、转向几何、按顺序经过原路标；起点附近路标后的前6m和后1m保留，以避免重新扰动已完赛起步段。
- 2先完成1的搜索，再进行扩展搜索；不会选择名义圈时更慢的候选。模型名义最优不等于实测最优，实际更慢必须回退。
- curvature_feedforward_gain默认.4、pursuit_lookahead_scale默认1.0保持smooth_pursuit原行为，可独立调参。本次没有选择强前馈作为通用最快默认，因为不同响应情况下它可能更慢。它们只作用于smooth_pursuit，不作用于race_feedback。
- 调试入口可显式选择lateral_control，默认仍smooth_pursuit。race_feedback是实验候选，不是已验证完赛默认；此前实测失败不能因为模型通过而被抹去。
- 所有路线保留.34m净空；.23m靠墙停车、.15m参考偏差停车、response_delay=.25、雷达/关节/TF/数据时效保护不改。不提高max_speed=10，不改官方车辆、地图或裁判。新增搜索只在静止规划阶段，运行中不重新改线。

## 筛选结果（模型，非官方）

| 配置 | 假设80ms响应 | 假设150ms响应 | 假设250ms响应 |
|---|---:|---:|---:|
| 原路线，smooth_pursuit，横向6.5 |85.82|87.94|106.72|
| 新路线，smooth_pursuit，横向6.5 |85.82|87.48|108.94|
| 新路线，race_feedback，横向6.5 |83.06|85.08|96.36|
| 新路线，race_feedback，横向7.5 |78.60|80.92|95.14|
| 新路线，race_feedback，横向8.5 |74.98|77.58|96.22|

以上其他预算固定acceleration6.6、braking3.6、yaw_rate3.8、反馈增益.25、提前启动释放true，恢复等待0。最后一组是本次保留候选中150ms情形最快，不是测得执行器响应为150ms，也不是所有现实情况最快。250ms情形7.5比8.5更快，所以不存在无条件最优。

部分原转向配置直接提高横向/制动/加速度预算后，在250ms模型逼近或触发.15m停车，已排除这些组；不会用未完成五圈的低时长冒充成绩。race_feedback横向9.5虽在部分模型更快，但较慢响应下偏差约.137m，过于接近保护线，未列为推荐候选。保留8.5组在三种模型都完赛，峰值偏差最大约.128m、车中心墙距最小约.285m；这不是翻车、轮胎侧滑和碰撞实测证据，正式试验仍逐步升档。

代码94项针对性回归通过：运行适配52、smooth pursuit14、time_attack9、投影3、前馈参数4、恢复/提前启动7、多尺度路线5。Python3.8语法和XML有效。真实ROS Noetic解析新路线、控制器和调参参数成功。模型完整记录见RACE_LINE_SEARCH_RESULTS.json。

## 更新

先stop并结束旧轮，停止旧控制器、场景、裁判。更新失败就不继续：
```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git switch main && git pull --ff-only && cd ~/ican_race_ws && catkin_make
source ~/ican_race_ws/devel/setup.bash
```
本地修改阻止更新时不要强制reset。各新终端加载两个setup。

## 每轮共用流程

启动命令包含官方场景、裁判、控制器。不要同时保留旧场景或速度发布节点。等待Ready；裁判检测环境、绑定、准备发车后只start一次：
```bash
rosservice call /tianracer/continuous_race/start '{}'
```
官方五圈锁定后stop；风险限速、明显超目标、偏差接近15cm、保护停车、翻车/碰撞也立即stop，失败不继续start：
```bash
rosservice call /tianracer/continuous_race/stop '{}'
```
每个候选只改一个变量。不需要连续三次。有效5/5、last_fault=null、无翻车/碰撞/保护停车且准确官方时间更短才保留；更慢或失败回退当前最佳。如果当前最佳参数不同于下面基线，不把它覆盖，单独标记本轮候选。<60秒必须官方有效结果original time<60000000微秒。

## 最短实测顺序

以下以已经完赛的smooth_pursuit、提前释放true、恢复等待0、预瞄.10为基线。若你已保存的最佳参数不同，保留它，不用本表代替官方最好记录。

R1只换路线：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true line_search_level:=2 run_report:=$HOME/ican_race_logs/fast_R1.json
```

R2相对R1只换控制器。如果R1更慢，先回退line_search_level=0，再独立只比较控制器，不为了路线优化强行保留更慢配置：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true line_search_level:=2 lateral_control:=race_feedback run_report:=$HOME/ican_race_logs/fast_R2.json
```

R2有效后，横向预算每次只提高0.5：6.5 -> 7.0 -> 7.5 -> 8.0 -> 8.5。不要一次跳完。预算提高只改变规划假设，不证明真实轮胎/侧滑/翻车安全。

7.0示例：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true line_search_level:=2 lateral_control:=race_feedback lateral_accel:=7.0 run_report:=$HOME/ican_race_logs/fast_lat70.json
```
后续依次把lateral_accel改成7.5/8.0/8.5，报告名相应改变。其他预算固定6.6/3.6/3.8，不同时增加braking、yaw_rate或反馈增益。

完整最快保留模型候选（只有逐步验证有效、各轮无风险且更快才到这一组）：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true line_search_level:=2 lateral_control:=race_feedback lateral_accel:=8.5 acceleration:=6.6 braking:=3.6 yaw_rate:=3.8 steering_feedback_gain:=0.25 run_report:=$HOME/ican_race_logs/fast_lat85.json
```

## 回退

首先保留你自己已验证的最快组。恢复原完赛算法、原路线的参考命令：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true line_search_level:=0 lateral_control:=smooth_pursuit lateral_accel:=6.5 speed_recovery_hold:=0 steering_preview_time:=0.10 run_report:=$HOME/ican_race_logs/fast_fallback.json
```

如果某轮只是更慢但仍安全，不必重做三个连续轮次；恢复实际最佳再做下一项独立对照。如果新算法首弯又出现风险，停止该算法试验，用已完赛的smooth_pursuit继续，只做路线/横向预算小步对照。

不会声称deadline前已经完成官方60秒目标；本次成果是可运行的路线搜索、模型筛选和完整回退入口。
