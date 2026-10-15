# 本版验证、复现与未验证项

- ROS Noetic / Python3.8.10 / NumPy1.21.6 / SciPy1.10.1：338项完整回归通过，见NOETIC_TESTS.txt。包含原传感器/激光/地图/硬偏差保护、跨圈、启动、延迟反馈、真实rosbag API往返，以及新增路线接受/拒绝、峰值质量门控、单变量继承、预算显式入口等检查。
- 完整回归后仅补充了依赖版本日志和YAML读取错误提示；参数解析/继承检查再次通过。后续每轮自动记录library_versions.json，用于复现优化器版本。
- MODEL_RESULTS.json：25个假设自行车/一阶执行器案例，20完成、5个250ms响应案例失败。保留全部失败，不将模型完成认证为官方5/5。
- NOETIC_MODEL.json：在原生Noetic复核前两组响应假设，共10个案例，均完成。优化器跨NumPy/SciPy版本可能选中略不同几何，不能声称字节相同：例如同预算算法80ms案例主环境65.88s、Noetic65.90s。应固定版本并保存实际planned_course.csv与SHA复现。
- 真实roslaunch参数解析：新功能开启及legacy回退均保留0.34m路线净空、0.25s响应预留、0.15m偏差停车、3.7制动和原转角变化率/启动设置；legacy_control=true关闭新路线速度搜索。
- 源码差异未涉及车辆/赛道/裁判/校验、loop_planner.py、predictive_follower.py和braking_envelope.py。受保护的传感器/激光/地图函数原有哈希检查通过。
- CHANGES.patch是零上下文补丁，只包含代码/launch/test变更；推荐更新分支。单独检查应用需git apply --unidiff-zero --check，不要覆盖未提交用户改动。

## 未验证

没有用户实际70秒bag/报告在本环境中的独立核验；模型代理不是70秒配置的保证。没有新算法或更高预算的Gazebo官方五圈、真实无翻车/碰撞、无保护停车/卡住/紧急制动、真实制动能力或摩擦极限验证。没有官方小于60秒结果。本版最小已列假设模型时间仍大于60秒。

预算10.1/10.2是按本次用户提高速度预算要求提供的受限试验，不是已实测安全范围；旧默认9.94门槛保留，只有显式budget_trial=true才扩展，制动3.7门槛不扩展。其他硬保护不减弱。

几何峰值门控不保证实际跟踪误差不变大。较大的定位或关节延迟下，模型误差/单周期降速仍有恶化；现场出现风险必须stop并回退，不能因模型用时更短而继续。

## 复现命令

```bash
cd ~/ican_race_ws/src/tianracer
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_*.py' -v
```

不启动车辆的离线模型对照：

```bash
rosrun tianracer_gazebo speed_envelope_audit.py \
  --output "$HOME/ican_race_logs/speed_envelope_model.json"
```

快速前两种响应假设：

```bash
rosrun tianracer_gazebo speed_envelope_audit.py --quick \
  --output "$HOME/ican_race_logs/speed_envelope_model_quick.json"
```

审阅代码：

```bash
git diff aa32de9..HEAD -- tianracer_gazebo/scripts tianracer_gazebo/launch tianracer_gazebo/test
```

真实官方验收仍按RUN_CN.md，必须有效5/5、原始官方结果、last_fault=null、无事故和异常；控制器run_report的official_result=false不可改写为认证。
