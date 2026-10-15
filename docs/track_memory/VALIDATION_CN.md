# 验证记录及限制

1. ROS Noetic / Python3.8.10 / NumPy1.21.6 / SciPy1.10.1：完整315项测试通过，日志见 `NOETIC_TESTS.txt`。包括原安全传感器/激光、保护函数源码哈希、规划、启动及圈缝回归；新增缓存等价/失效、prepare deepcopy、硬偏差停车、自适应预瞄状态和路线优化接受门控测试。
2. 默认配置与修改前父版本：五圈范围2500个确定反馈样本，控制输出及所有原有报告字段逐项一致。缓存开关独立测试也保持五圈输出、进度、误差逐项一致。
3. 真实roslaunch参数解析：新路线与legacy回退均验证原71预算、0.34m路线净空、0.25s响应预留、0.15m参考停车、2.5m连接长度等不变；即使同时请求全部新功能，legacy_control=true也强制关闭新路线、缓存和自适应预瞄。
4. `PREVIEW_MODEL.json`：20个假设模型案例，17个完成，3个不完成；其中缓存与原版包含相同失败。注意具体统计应按文件中的completed字段读取，不能把失败删掉。该文件保留自适应预瞄更慢的结果，不选报最好值。
5. `ROUTE_MODEL.json`：15个假设模型案例，12个完成、3个不完成。250ms执行器+40ms位姿/关节年龄组，原路线及两档新路线都跟踪失败；没有对此降低任何保护。
6. `NOETIC_ROUTE_MODEL.json`：同地图、同预算，在Noetic对原路线/强度0.5/1.0做两组响应假设的6案例复核，均完成，选线哈希和用时与主环境对应案例一致。不是Gazebo实测。
7. 受约束全局平滑之前，还试过24组平滑+墙距投影候选，没有通过原完整几何可行性检查，未接入行驶。随后采用带周期法向基函数的约束二次目标，仍由原几何与速度包络检查接受。更强的后续离线重复优化未作为本版自动行驶功能。

模型限制：自行车运动学和人为设定的一阶响应，不含轮胎侧滑/翻车、完整车体接触、Gazebo物理、AMCL跳变、真实TF/激光数据链和官方裁判。完整测试通过不等于不碰撞或正式有效5/5。指令降速较大处必须现场核对，不可称已消除紧急制动。

未验证：原71秒成功bag对应的执行器参数与限速占比、新路线的官方五圈、真实制动裕量、无翻车/碰撞/保护停车/卡住、官方小于60秒或55秒。

## 可复现检查

```bash
cd ~/ican_race_ws/src/tianracer
python3 -m unittest discover -s tianracer_gazebo/test -p 'test_*.py' -v
```

赛后、未启动车辆时做模型对照（可能需要数分钟）：

```bash
rosrun tianracer_gazebo track_memory_audit.py \
  --route-only --output "$HOME/ican_race_logs/route_model_check.json"
```

或包括缓存/预瞄对照：

```bash
rosrun tianracer_gazebo track_memory_audit.py \
  --output "$HOME/ican_race_logs/memory_preview_route_model_check.json"
```

审阅代码差异：

```bash
git diff aaea1a5..HEAD -- tianracer_gazebo/scripts tianracer_gazebo/launch tianracer_gazebo/test
```

`CHANGES.patch` 是零上下文代码补丁；推荐直接按说明更新分支。若单独检查/应用补丁，需要 `git apply --unidiff-zero`，并先使用 `--check`。

`BASELINE_MODEL_SENSITIVITY.json` 只用于解释同路线模型灵敏度，不是提高braking/lateral_accel、降低response_delay或保护阈值的授权与安全证明。
