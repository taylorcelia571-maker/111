# 投影一致性修复与官方验证

本次修复针对源码中可复现的两处几何不一致，不宣称已证明用户首弯故障完全由它们引起。

1. ClosedPath.project 原先在搜索窗口之外计算最近点和距离，再单独夹紧返回进度。改为先限制线段与投影比例，使返回位置和偏差对应同一点。
2. PredictiveFollower.step 原先把圈数用的只增不减进度直接用于局部控制参考。局部定位沿路线后移时，连接线欧氏误差会包含沿路线距离。改为用实际局部投影计算控制参考、切线、目标及曲率前馈；圈数进度仍单调，速度包络索引不变。
3. 报告增加 control_pose、tracking_reference、tracking_tangent、projected_progress、previous_progress、signed_tracking_error、controller_revision。

合成回归：连接线上沿路线回调8cm，旧控制器报告约8cm控制误差，新控制器误差接近零。该回归不代表真实AMCL发生了这种回调，也不代表侧滑/碰撞已解决。

保留 launch 默认值、A组预算、0.34m净空、0.23m墙距停车、0.15m参考偏差停车、雷达、时效与启动释放条件。未改官方地图、车辆、裁判或订阅模型真值。

## 验证证据

完整发现225项：224项执行通过，test_waypoint_laps 因当前Python环境缺少 actionlib_msgs 无法导入。新增3项另行复验通过；运行保护49项、time_attack9项、smooth_pursuit14项通过，包含在224项中，不重复相加。

没有ROS/Gazebo实车动力学验证，没有收到A_control.bag原文件。用户附件仅包含bag元信息和报告。模型、测试、官方成绩必须分开。当前最佳仍按用户报告的Run2约82秒，准确微秒值及对应参数需官方原始记录确认。未验证60秒目标。

## 一轮完整操作

失败轮先stop，保存原报告、规划文件、status、bag和日志。停止旧控制器、裁判、仿真，再更新：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git status --short
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

若pull提示本地修改或分叉，停止更新并保存报错；不要reset或覆盖本地文件。rospack应指向本工作空间。每个新终端加载两个setup。

终端A重新生成静止默认起点：
```bash
roslaunch tianracer_gazebo planned_race_demo.launch
```
终端B：
```bash
roslaunch tianracer_sim_referee referee.launch
```
终端C，仅A组，不同时提高预算或改变启动方式：
```bash
roslaunch tianracer_gazebo time_attack_race.launch lateral_accel:=6.5 acceleration:=6.6 braking:=3.6 yaw_rate:=3.8 steering_feedback_gain:=0.25 run_report:=$HOME/ican_race_logs/A_projection_v1.json
```
等待Ready；新终端检查cmd_vel只有一个发布者，status反馈有效且未active：
```bash
rostopic info /tianracer/cmd_vel
rosservice call /tianracer/continuous_race/status '{}'
```
本轮沿用失败A组的启动流程，不额外增加prepare，以避免改变第二个变量。

终端E，发车前录制；若amcl_pose名称不同，以rostopic list为准替换。不要record -a：
```bash
rosbag record -O ~/ican_race_logs/A_projection_v1.bag /clock /tianracer/cmd_vel /tianracer/odom /tianracer/joint_states /tianracer/amcl_pose /tianracer/scan /tf /tf_static /rosout
```
裁判检测、绑定、准备发车后，只start一次：
```bash
rosservice call /tianracer/continuous_race/start '{}'
```

出现风险限速、明显超目标速度、偏差接近15cm、翻车/碰撞、停车立即结束，不为采集数据继续驾驶。官方五圈锁定后也stop：
```bash
rosservice call /tianracer/continuous_race/stop '{}'
rosservice call /tianracer/continuous_race/status '{}'
```
stop后保存status，Ctrl+C正常结束bag。保留JSON、规划CSV/JSON、完整C日志、bag和官方每圈/总时间原始记录，下一轮前归档，不能覆盖。

检查departure_trace中的 controller_revision=projection-consistency-v1。重点对比原故障启动后约0.9–1.3秒的控制误差、参考误差、投影进度及位姿；以及首弯约3秒的墙距停车、转向反馈、雷达风险限速。报告诊断只是约20Hz采样，真实响应分析用bag。

安全验证：官方有效5/5，无翻车/碰撞/保护停车，last_fault=null。新最佳必须官方时间比Run2准确原始时间更小；不要求连续3次。仅安全完赛但更慢不能替代最佳。60秒必须官方有效结果original time<60000000微秒。

## 失败后恢复旧代码

只在工作树干净、旧程序已停止时使用；保留旧代码和新代码测试结果分别标记：
```bash
cd ~/ican_race_ws/src/tianracer
git switch --detach fcdb73e0ae01fe6ed37b09f99d607ed1582bbe11
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
```
重新生成默认起点；如果附件2确认为82秒Run2，恢复其7.0/7.0/3.8/4.0/.25参数。否则先确认对应关系，不把A组称为82秒配置：
```bash
roslaunch tianracer_gazebo time_attack_race.launch lateral_accel:=7.0 acceleration:=7.0 braking:=3.8 yaw_rate:=4.0 steering_feedback_gain:=0.25 run_report:=$HOME/ican_race_logs/best82_restored.json
```
如需重新进入新版，工作树干净时git switch main，然后git pull --ff-only并重编译。
