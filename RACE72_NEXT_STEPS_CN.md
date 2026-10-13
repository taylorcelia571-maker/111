# 72秒之后：起步与弯道入口/出口路线优化

当前基线是用户报告的72秒：startup_release_early=true、line_search_level=2、race_feedback、lateral_accel=9.9；其他预算默认acceleration6.6、braking3.6、yaw_rate3.8、反馈增益.25、预瞄.10、规划legacy。没有收到本轮裁判原始结果，本页不把报告模型当官方成绩，也未验证60秒达标。

## 改动

* line_search_level=3：先保留2级全部搜索，再分别优化弯道入口、弯心和出口，偏移中心向前/后移动0.5m。每个候选仍重新验证整条路线.34m净空、转向可行性、路标顺序。起步附近已有保护段保持不变，搜索只在静止规划时进行。
* startup_acceleration在race_tuning/demo入口开放，默认仍2.0。可以独立试3.0，不能解除启动就绪检查：至少.6m、误差<2.5cm、航向/转角误差<.05rad持续.3s。它只可能改善首圈少量时间，不能单独填补12秒差距。
* heading_feedback_gain/cross_track_gain开放，默认仍.8/.8。更强航向反馈在慢响应模型能改善跟踪，但有时更慢，不默认增大。
* speed_planning开放，默认仍legacy。delayed_braking保留.25秒响应预留，不是降低response_delay；模型收益小且存在不利情形，因此仅独立比较。
* 0.34m净空、0.23m墙距停车、0.15m偏差停车、雷达、关节、TF及数据时效、转角变化率等不改。官方场景、车辆、裁判不改。实际赛道弯道与检查点不能跳过，只减少参考线多余摆动并平滑弯道衔接。

## 模型证据（不是预计官方成绩）

|配置|80ms假设响应|150ms假设响应|250ms假设响应|
|---|---:|---:|---:|
|72秒参数对应模型、路线2|71.54|74.82|99.90|
|仅路线3|69.72|73.00|96.90|
|路线2，仅启动加速度3|71.40|74.66|99.78|
|路线2，仅yaw_rate4.0|70.94|74.04|99.84|

第一行与实测72接近不证明实际响应是80ms。模型不能测量执行器延迟，不能证明不翻车/侧滑/碰撞。

探索更高横向预算13/14的部分组合在250ms模型失败，因此未推荐；其中某些快响应模型时间接近60，但不能作为达标成绩。没有保留候选证明官方<60秒。

95项针对性回归通过，含路标顺序、整路线净空、启动区域保留、入口/出口搜索不增加名义圈时、参数范围、启动与停车保护。Python3.8/XML有效。ROS Noetic原生参数解析确认路线3、启动加速度和反馈/规划参数正确转发。模型完整结果见RACE72_MODEL_STUDY.json。

## 更新并只换路线

先stop、结束当前轮、停止旧控制器/场景/裁判。更新失败不能继续编译启动：
```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git switch main && git pull --ff-only && cd ~/ican_race_ws && catkin_make
source ~/ican_race_ws/devel/setup.bash
```
若本地修改阻止拉取，不reset强制覆盖。

首轮与72秒配置相比只把路线2改为3：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true line_search_level:=3 lateral_control:=race_feedback lateral_accel:=9.9 run_report:=$HOME/ican_race_logs/route3_99.json
```
该命令包含官方场景和裁判，不要同时启动旧节点。更强搜索在发车前完成，等待Ready。裁判检测、绑定、准备后只start一次：
```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
rosservice call /tianracer/continuous_race/start '{}'
```

## 下一项只动一个变量

路线3有效且更快才保留；否则恢复2。以实际最快路线为准，下面示例假定3通过。

启动参数单独对照：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true line_search_level:=3 lateral_control:=race_feedback lateral_accel:=9.9 startup_acceleration:=3.0 run_report:=$HOME/ican_race_logs/start30_99.json
```
更快才保留startup_acceleration=3，反之2。不要同时增加startup_speed或减少启动释放条件。

yaw单独对照路线3默认启动参数（不是从启动3组直接切换来做单变量归因）：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true line_search_level:=3 lateral_control:=race_feedback lateral_accel:=9.9 yaw_rate:=4.0 run_report:=$HOME/ican_race_logs/yaw40_99.json
```
4.0安全有效且更快后再试4.2，每次只改该值。模型显示4.5到5.0没有进一步收益，不默认继续提高。

制动可独立从3.6到3.8再4.0比较，但它是规划能力假设，不代表实际制动能力增强。不要同时提高加速度。正常acceleration从6.6提高到8在模型几乎无收益，不作为主要手段。

## 成功/失败与恢复

按单次最快规则：官方有效5/5、last_fault=null、无翻车/碰撞/保护停车且官方准确总时间更短才替换最佳，不要求连续三次。当前最好按用户报告72秒保留，准确时间以裁判为准。小于60秒必须官方original time<60000000微秒与有效结果。

风险限速、明显超目标速度、参考偏差接近15cm、翻车/碰撞或控制停车立即stop；不在故障位置重复start：
```bash
rosservice call /tianracer/continuous_race/stop '{}'
```
停止旧进程并重新生成默认静止起点，恢复已跑通72秒参数：
```bash
roslaunch tianracer_gazebo race_tuning_demo.launch startup_release_early:=true line_search_level:=2 lateral_control:=race_feedback lateral_accel:=9.9 startup_acceleration:=2.0 heading_feedback_gain:=0.8 cross_track_gain:=0.8 speed_planning:=legacy run_report:=$HOME/ican_race_logs/best72_restore.json
```
若需代码也回到本版之前，工作树干净且节点已停止时可git switch --detach b593076a8fb12d91ed650988d06364fc30d77bdd并重编译。此时再更新须先git switch main。

不把平均速度、参考圈、名义圈时或失败轮用时当官方5圈成绩。本版能否将实际72秒进一步缩短，需要这几项独立试验确认。
