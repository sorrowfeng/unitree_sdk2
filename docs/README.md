# docs —— 项目文档索引

> 本目录存放**跨会话需要保留**的说明性文档。
> 日常操作只需要看 `example/r1/high_level/` 下那几份（见下表"上机"一行）。

## 各文档管什么

| 文档 | 什么时候看 |
|---|---|
| **本文件** | 不知道去哪找的时候 |
| [`../AGENTS.md`](../AGENTS.md) | **项目记忆总纲**：目标、三台机器、目录、已修复清单（勿回退）、红线、约定 |
| [`../example/r1/high_level/R1_TELEOP_QUICKSTART.md`](../example/r1/high_level/R1_TELEOP_QUICKSTART.md) | **平时启动遥操就看这个**（五步，只有命令） |
| [`../example/r1/high_level/R1_TELEOP_RUNBOOK.md`](../example/r1/high_level/R1_TELEOP_RUNBOOK.md) | 出问题时看：判据、常见现象二分表、红线 |
| [`../example/r1/high_level/R1_TELEOP_STARTUP.md`](../example/r1/high_level/R1_TELEOP_STARTUP.md) | 完整上机手册：原理、日志解读、网络四方案、故障二分 |
| [`../example/r1/high_level/R1_BACKPACK_ARCHITECTURE.md`](../example/r1/high_level/R1_BACKPACK_ARCHITECTURE.md) | 背包/游标/灵巧手普查档案（环境实证） |
| [`../example/r1/high_level/R1_CONTROL_ALGORITHM.md`](../example/r1/high_level/R1_CONTROL_ALGORITHM.md) | 算法/参数与官方实现对照 |
| [`../example/r1/high_level/scripts/r1_hand/README.md`](../example/r1/high_level/scripts/r1_hand/README.md) | 灵巧手整套（自包含目录）的说明 |
| [`HISTORY.md`](HISTORY.md) | **为什么要这么设计** —— 里程碑、关键决策、踩过的坑（按时间） |
| [`ENVIRONMENT.md`](ENVIRONMENT.md) | 背包/VM 的环境细节与主机级坑（apt 坏、时钟漂移、fishros 菜单…） |

## 维护约定

- 新的事实**先落 `AGENTS.md`**（那是 AI 和人都先读的）；细节膨胀了再拆到本目录。
- 本目录的文件要能独立看懂，不要写成"见上文"。
- 不写凭据。背包/VM 密码只经环境变量传递。
