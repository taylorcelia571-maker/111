# 小车完赛恢复与提速参数对照（未验证官方60秒）

用户最新授权直接修改代码、参数并上传仓库，本版暂停压缩包分析。当前云容器具备ROS Noetic，但没有Gazebo11/gazebo_ros，也没有通往用户ROS2GO的连接。因此没有启动车辆或完成官方五圈，不能声称首弯问题已解决或小于60秒。

## 本版

新增 race_tuning.launch：明确选择历史已有87.546秒完赛记录对应的 smooth_pursuit/legacy 配置族，默认预算6.5/6.6/3.6/3.8，反馈增益0.25，max_speed仍为10。该入口与失败的race_feedback入口分开。

新增 race_tuning_demo.launch：一个命令启动原有官方场景、原有裁判、控制器。不会自动发车或修改裁判。用户仍需在裁判检测环境、绑定和准备后start一次。若已经单独启动场景/裁判，改用race_tuning.launch，不能同时运行两套。

新增 steering_preview_time：合法范围0.05–0.25秒，默认0.10保持原行为。只调整曲率前馈预瞄，既不降低response_delay，也不改变规划制动预算。较大预瞄可能补偿转向滞后，也可能提前切弯并更慢，必须单变量对照。参数通过planned_race转发，状态和规划/运行报告记载。

路线净空0.34m、墙距停车0.23m、偏差停车0.15m、转角/转角变化率、雷达、反馈时效、启动释放保护均保留。官方车辆、地图、裁判未修改。本次不把max_speed作为主要提速手段。

## 验证

78项针对性回归通过：运行保护49、smooth pursuit14、time_attack9、投影3、新预瞄3。Python3.8语法与XML有效。原生ROS Noetic的roslaunch --dump-params解析race_tuning.launch steering_preview_time:=0.15，核对了smooth_pursuit/legacy、6.5/6.6/3.6/3.8/.25，以及route_margin=.34、response_delay=.25、max_error=.15。

33次假设一阶自行车模型运行（包括重复基线对照），全部完成模型五圈。这不是Gazebo，缺少真实轮胎、侧滑、AMCL更新、翻车动力学与实际雷达，不能证明无碰撞、无风险限速或实际完赛。

同一模型在假设150ms响应时：

| 配置 | 模型五圈秒数 |
|---|---:|
| 默认恢复入口 |88.32|
| 仅lateral_accel=7.0 |86.46|
| 仅lateral_accel=7.5 |84.80|
| 默认预算，仅预瞄=.12 |88.84|
| 默认预算，仅预瞄=.15 |90.32|

假设250ms响应时预瞄从.10增加到.12，模型106.82降至105.44；该趋势并不适用于较快响应。不能宣称预瞄越大越快。模型也出现较大的瞬时指令降速，参数预算不代表实测执行器能力。见RACE_TUNING_MODEL_RESULTS.json。

## 更新

停止失败轮，stop后停止旧控制器、旧仿真、旧裁判；保存已有数据。不要从故障位置再start。

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

若pull或编译失败，停止；不要reset覆盖本地修改。rospack必须指向此工作空间。每个新终端加载同样两个setup。

## 先执行R0恢复试验

```bash
roslaunch tianracer_gazebo race_tuning_demo.launch run_report:=$HOME/ican_race_logs/R0.json
```

此命令已经包含官方场景和裁判，不再另开planned_race_demo或referee。等待Ready，确认日志Controller=smooth_pursuit；检查cmd_vel只有一个发布者。官方GUI完成环境检测、绑定、准备发车之后，在新终端：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

只start一次。无法起步、风险限速、速度明显超目标、偏差接近15cm、翻车/碰撞或停车，立即stop，结束本轮，不继续增预算：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
```

官方五圈成绩锁定后也stop。R0是恢复诊断，不宣称比现有82秒最佳更快。

## 单次最快候选

R0未安全完赛时不进入这些候选。每轮先结束旧进程、从官方静止起点重启场景，仍只start一次。不需要连续三次。除对应变量外不改变参数。

S1，相对R0只提高横向预算：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch lateral_accel:=7.0 run_report:=$HOME/ican_race_logs/S1.json
```

S2，相对S1只提高横向预算：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch lateral_accel:=7.5 run_report:=$HOME/ican_race_logs/S2.json
```

P1，与R0独立对照，只改变转向预瞄（不是从S2直接切换来做单变量归因）：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch steering_preview_time:=0.12 run_report:=$HOME/ican_race_logs/P1.json
```

P1只有在延迟明显且无几何切弯风险时才可能有益，模型显示也可能更慢，不默认采用.15或.18。

任何候选失败/更慢就回退原最佳，不自动累计多个参数。本版不宣称这些候选能到60秒。成功判据：官方有效5/5，last_fault=null，无翻车/碰撞/保护停车，准确官方时间小于现有最佳。现有82秒应使用Run2准确原始微秒值，不能只拿四舍五入值比较。小于60秒必须官方original time<60000000微秒。

## 回退

恢复本版R0：结束旧控制器、仿真、裁判后重启R0命令，预瞄默认为.10。

若需恢复之前82秒所在版本，工作树干净且进程全部停止时：
```bash
cd ~/ican_race_ws/src/tianracer
git switch --detach fcdb73e0ae01fe6ed37b09f99d607ed1582bbe11
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
```

重新启动当时的官方场景、裁判。只有附件2确认为Run2时，才将其7.0/7.0/3.8/4.0/.25称为82秒参数：
```bash
roslaunch tianracer_gazebo time_attack_race.launch lateral_accel:=7.0 acceleration:=7.0 braking:=3.8 yaw_rate:=4.0 steering_feedback_gain:=0.25 run_report:=$HOME/ican_race_logs/best82_restored.json
```

本版没有在用户电脑自动启动比赛、没有自动提交成绩、没有更改官方计时。
