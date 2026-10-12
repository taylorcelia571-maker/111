# rapid_race 第二圈靠墙停车修正

本次用户日志在仿真时间 132.242 秒、约 1.36 个参考闭环后停止：`Localized vehicle too close to static wall`。实际车辆中心净空 0.2288 m，小于原 0.2300 m 阈值；TF 年龄仅 0.019 秒。本次是主动墙距保护停车，不是日志证明的计算卡死。里程计还有 -0.131 m/s 横向速度、转向关节跟踪误差和实测速度高于指令的现象，不能仅凭文本判断侧滑/定位/执行器各自影响。

此前墙距修正只给 optimized_race 加了 0.34 m 路线净空，rapid_race 未声明和转发 route_margin，仍采用 0.26 m，仅比停车阈值多 3 cm。这个入口遗漏已修复：rapid_race 默认 route_margin=0.34，并传给实际规划器。保留 0.23 m 墙距停车、0.15 m 跟踪偏差、激光及时间戳保护。没有修改官方地图、模型、裁判或 PID。

## 更新和重新发车

先停车、在裁判结束失败轮，退出旧控制、仿真和裁判。不能从靠墙位置反复 start。更新时保留本地改动，不使用 reset --hard：

```bash
source /opt/ros/noetic/setup.bash
cd ~/ican_race_ws/src/tianracer
git pull --ff-only
cd ~/ican_race_ws
catkin_make
source ~/ican_race_ws/devel/setup.bash
rospack find tianracer_gazebo
```

每个新终端都先分别执行两行 source：

```bash
source /opt/ros/noetic/setup.bash
source ~/ican_race_ws/devel/setup.bash
```

A 重启官方场景恢复默认静止起点：

```bash
roslaunch tianracer_gazebo planned_race_demo.launch
```

B 重启裁判，检测官方环境并确认正式测试绑定有效：

```bash
roslaunch tianracer_sim_referee referee.launch
```

C 保留本次速度预算，只改变路线净空，便于确认此次修正效果；此预算不是已认证的车辆极限，不保证下一轮一定完赛：

```bash
roslaunch tianracer_gazebo rapid_race.launch max_speed:=10.0 lateral_accel:=6.5 acceleration:=5.5 braking:=3.0 yaw_rate:=3.8 lateral_control:=smooth_pursuit route_margin:=0.34 start_blend:=false response_delay:=0.25 predict_pose:=false refine_line:=false
```

等待 Ready 后，最低中心参考线净空必须至少 0.34 m，不能仍显示 0.261 m。D 核对并静止预转向：

```bash
rosparam get /tianracer/continuous_race/route_margin
rosservice call /tianracer/continuous_race/prepare '{}'
```

应分别为 0.34、success=True。车体保持静止，裁判点击“准备发车”，然后 D：

```bash
rosservice call /tianracer/continuous_race/start '{}'
```

裁判显示有效五圈、结果锁定后立即停车，不必等第六个参考闭环：

```bash
rosservice call /tianracer/continuous_race/stop '{}'
```

若再停住，保持 C 打开，保存 ERROR、下面 status 和本轮两个 JSON。不要通过再次 start 覆盖故障，也不要降低停车阈值：

```bash
rosservice call /tianracer/continuous_race/status '{}'
cat ~/ican_race_logs/control_run_report.json
cat ~/ican_race_logs/planned_course_report.json
```

## 另一种提速方法：在有墙距余量的路线内逐弯细调

先用上述路线连续验证三次有效五圈，再只把 C 命令中的 `refine_line:=false` 改为 `refine_line:=true`，其他参数和起点保持相同。该优化在静止阶段完成，比较全圈曲率/速度/响应预留的时间代理，同时检查连续地图净空及原航点顺序；运行中不在每圈冲线重新规划/停车。航点检查不替代裁判检测线，仍需有效五圈。

本次真实仓库地图的离线模型结果（同一 10/6.5/5.5/3/3.8 预算、smooth_pursuit、0.25 秒预留、新鲜关节反馈）为：

| 假设执行器响应 | 0.34 m 路线 | 逐弯细调 | 细调最小车辆中心净空 |
| --- | ---: | ---: | ---: |
| 80 ms | 89.08 s | 87.34 s | 0.3254 m |
| 120 ms | 91.20 s | 89.08 s | 0.3281 m |
| 200 ms | 108.46 s | 105.98 s | 0.3405 m |

这些是五个假设自行车模型参考闭环，不含实际轮胎动力学、AMCL 和官方检测线；不能声称是真实完赛或保证稳定。机器可读结果见 RAPID_WALL_OFFLINE_RESULTS.json。新路线增加余量，模型比旧窄路线更慢；本次修正针对失败完赛，并没有实现一分钟目标。逐弯细调在保留余量的条件下有数秒模型收益，须再核对官方实测中位数和完赛率。

日志明确 max_speed=10 不起限制，计划峰值约 6.07 m/s；设为 11 无模型收益。起步实测运动只需 0.083 秒，随后大约 8–10 秒仍在 departure/纠偏，问题是跟踪建立，而非启动服务迟缓。继续提速应从同步的转角指令/关节实际转角、速度指令/里程计、偏航/横向速度和定位数据标定响应，不能直接提高弯道预算或减少响应预留。完整录包与验收流程见 [RACE60_CURRENT_VALIDATION_CN.md](RACE60_CURRENT_VALIDATION_CN.md)。不用额外订阅 Gazebo 真值。
