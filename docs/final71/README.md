# 最后一轮71秒基线优化

独立分支time_attack_speedup_final；main保持原样。

- [逐步更新、实验、官方验证及回退命令](EXPERIMENTS_CN.md)
- [真实开发检查、模型结果与限制](VALIDATION_CN.md)
- [模型全部25案例](MODEL_RESULTS.json)
- [冻结71秒基线](BASELINE_71.json)
- [完整代码diff](FULL_CODE_DIFF.patch)
- [Noetic完整回归](NOETIC_TESTS.txt)
- [最终新增检查](FINAL_TESTS.txt)

优先：先复验新分支baseline，再只试V2路线评分。V1候选模型收益很小，不同时提高速度预算。V3只有V1/V2现场有效后才合并。

不得把模型结果或控制参考圈数写成官方成绩；一分钟目标未验证。
