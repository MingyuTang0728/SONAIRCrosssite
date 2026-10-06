# SONAIR Sim2Real Benchmark：文献依据与实验设计

> 目的：用真实 UR5e 采集的数据，建立一个 **sim 与 real 之间的标准评判尺**。
> 未来的用户把自己的仿真模型（或修正模型）放进来，就能得到一张可比较的"差距成绩单"。
>
> 本文分四部分：① 文献里的机器人 benchmark 是怎么做的；② 由此得出的设计原则；
> ③ 针对本平台的实验设计（采什么、做哪些实验）；④ 现状评估和下一步。

---

## 1. 文献：机器人领域的 sim2real benchmark 怎么做

已有工作大致可以分成四类。SONAIR 要做的"桥梁"，正好需要把它们组合起来。

### 1.1 直接量化"现实差距"：同一指令，比较 sim 和 real 的轨迹

- **Collins, Howard, Leitner (ICRA 2019)，*Quantifying the Reality Gap in Robotic Manipulation Tasks*。**
  在 Kinova 机械臂上用动作捕捉记录真值，把同一组操作任务放进 MuJoCo、PyBullet、V-REP 等仿真器，
  逐一比较轨迹。这是与 SONAIR 最接近的先例：**同一指令 → 两边的轨迹 → 差距**。
  局限：只比较仿真器本身，没有提供"让别人提交模型来打分"的机制，也没有测量地板（real 对 real 的可重复性）。
- **Collins et al. (2020)，*Traversing the Reality Gap via Simulator Tuning*。**
  用优化方法调整物理引擎参数，让仿真轨迹逼近真实数据集，表明差距的一大部分可以靠参数辨识消掉。
- **Erez, Tassa, Todorov (ICRA 2015)，*Simulation Tools for Model-Based Robotics*。**
  在多个引擎里运行同一模型，并提出定量的仿真性能指标。启示：要比较的东西，必须是同一个模型、同一组指令。
- **执行器辨识（PACE 类方法）。** 一篇 2026 年的工作在 UR7e 上用 chirp 轨迹激励，以 CMA-ES 辨识
  摩擦、armature 和电机延迟：关节空间 RMSE 从默认参数下的约 7° 降到 2° 以下。
  这说明 **UR 系列的差距主要来自执行器和摩擦模型**，与我们发现的 MuJoCo menagerie 伺服滞后 0.2 s 一致。
- **Clochiatti et al. (Robotica 2024)，*Electro-mechanical modeling and identification of the UR5 e-series robot*。**
  UR5e 的动力学与电气参数辨识，可直接作为"物理模型基线"的参数来源。

### 1.2 把"预测真实系统"做成公开 benchmark：提交模型、统一打分

- **Weigand et al. (2022)，*Dataset and Baseline for an Industrial Robot Identification Benchmark*（KUKA KR300）。**
  公开了正、逆动力学辨识数据集、原始高频数据、视频、**基线和统一的评价指标（figures of merit）**，
  让不同辨识方法可以直接比较，并已集成进 deepSI 库。
  **这是 SONAIR "用户放入模型 → 按我们的尺打分" 最直接的模板。**
- **Real Robot Challenge（Bauer et al., 2022；TriFinger 集群）。**
  用户远程提交代码，在真实机器人上自动执行并返回数据；同时发布了数百小时的比赛数据集。
  启示：评测流程要完全自动化，提交与评分走同一条代码路径。

### 1.3 衡量"仿真结论能否迁移到现实"：预测性指标

- **Kadian et al. (RA-L 2020)，*Sim2Real Predictivity*。** 提出 **SRCC**（Sim-vs-Real Correlation Coefficient）：
  如果方法 A 在仿真里比 B 好，现实里是否也好？Habitat 挑战赛原始设置的 SRCC 只有 0.18，
  调整仿真参数后升到 0.844。
  启示：**一个 benchmark 有没有用，要看它的排序能否被现实复现**，而不只是看误差小不小。
- **SIMPLER (Li et al., CoRL 2024)。** 用 1500 多次 sim/real 配对评测，验证仿真环境能否代替真机评测，
  指标是 Pearson 相关和 **MMRV（Mean Maximum Rank Violation，最大排序违背均值）**。
  指出两类主要差距：**控制差距**和视觉差距。
- **NVIDIA (RSS 2025 Workshop)，*Robot Policy Evaluation for Sim-to-Real Transfer: A Benchmarking Perspective*；
  REALM (2025)。** 主张系统地加大扰动和任务难度来测鲁棒性，并量化 sim 表现与 real 表现的对齐程度。
  REALM 强调"对齐的机器人控制"是 sim/real 相关性高的前提。

### 1.4 修正模型：未来用户最可能提交的东西

- **Golemo et al. (CoRL 2018)，Neural-Augmented Simulation。** 用 RNN 学习 sim 轨迹与 real 轨迹之差，
  再把它叠加到仿真器上。这正是本 benchmark 的"提交模型"形态：输入 sim 轨迹和指令，输出 real 轨迹。
- **Grounded Action Transformation (Hanna & Stone, AAAI 2017)**、**TossingBot 残差物理 (Zeng et al., RSS 2019)**：
  在物理模型上学习残差。
- **Lutter et al. (ICRA 2021)。** 在真实 Barrett WAM 上比较黑箱模型与物理模型：
  黑箱模型在数据分布内拟合好，**分布外会发散**；物理模型外推更好。
  这为"整格留出（held-out cells）+ 分布外任务"的测试设计提供了直接依据。
- **综述：Zhao et al. (2020)，*Sim-to-Real Transfer in Deep RL for Robotics: a Survey*。**

### 1.5 文献的空白，也就是 SONAIR 的贡献点

| 已有工作 | 做到了 | 缺少的 |
|---|---|---|
| Collins 2019 / Erez 2015 | 同指令下比较 sim 和 real 轨迹 | 没有提交与打分机制，没有测量地板，没有尾部指标 |
| Weigand 2022 | 公开数据集、基线、统一指标 | 不是 sim2real，没有仿真对照，只看内插 |
| SRCC / SIMPLER / REALM | 验证仿真排序能否预测现实 | 指标是任务成功率，不是动力学和传感器层面的差距 |
| NAS / 残差物理 | 修正模型 | 每篇用自己的数据和指标，无法横向比较 |

**SONAIR 的定位：** 一个**动力学 + 惯性传感器层面**的 sim2real 基准。它有三个特点：
- 每个结论都带着**测量地板**；
- 按**整格留出**考察跨工况泛化，并**以尾部（p95）为头条**；
- **用 SRCC/MMRV 证明基准本身的排序在现实中成立**。
这三点在上表中没有同时出现过。

---

## 2. 由文献得出的七条设计原则

1. **给两边同一个指令，而不是同一个结果**（Collins 2019；`target_q` 回放，已实现）。
   回放实测轨迹会让差距恒为零。
2. **每个差距数字都带地板**：real 对 real 的重复误差和测量误差预算。差距不显著大于地板就不能发表
   （Gate B，已实现）。
3. **训练、验证、测试三层分开**：辨识数据 → 工况内重复 → 整格留出 → 分布外任务
   （Weigand 的辨识集；Lutter 的分布外结论）。
4. **头条指标看尾部**：中位数和 p95 并列，p95 为主，再辅以分布距离（Wasserstein）。
5. **提交与基线走同一条代码路径**：至少有 identity、常数偏移、辨识过的物理仿真、学习型残差四档基线
   （Weigand、Real Robot Challenge）。
6. **验证基准本身**：差距必须随工况系统性变化（Gate C）；不同方法的排序要有置信区间；
   sim 的排序要能被 real 复现（SRCC/MMRV）。
7. **协议可移植**：别人用自己的机器人照同一协议采集，就能得到同一格式的成绩单。

---

## 3. 实验设计

### 3.1 基准的三个赛道（用户能"放进来"的三种东西）

| 赛道 | 用户提交什么 | 我们怎么做 | 回答的问题 |
|---|---|---|---|
| **A：仿真器保真度** | 一个仿真模型配置（MJCF/USD + 参数），或用我们的指令集跑出的仿真轨迹 | 用全部测试指令回放，与真机对比 | "你的仿真离真机多远？" |
| **B：sim→real 修正** | 一个修正模型 f(sim 轨迹, 指令) → real 轨迹 | 在留出工况和分布外任务上打分（GCR） | "你的模型能把仿真拉近现实多少？" |
| **C：自带机器人（第二阶段）** | 按 SONAIR 协议采集的自有机器人数据 + 仿真 | 用同一套评分程序生成同格式的成绩单 | "你的机器人/仿真组合的差距有多大？" |

赛道 A、B 用本平台的 UR5e 数据作为参考集，可以排行。赛道 C 让基准成为通用标尺，
但不同机器人之间只比较归一化指标（相对地板的倍数、GCR），不比较绝对毫米数。

### 3.2 要采的数据：五组实验

#### E0 测量地板（必须先做，约半天）

| 内容 | 方法 | 产出 |
|---|---|---|
| IMU 噪声与零偏 | 静止记录 1 小时（`phase0`） | 噪声底、零偏 |
| 重力/尺度检查 | 六面翻转 | 加速度计尺度误差 |
| IMU 时间与安装 | **imu mount cal** 任务（已实现） | 延迟（约 100 ms）、安装旋转（约 −90° 绕 z） |
| 运动学地板 | 名义 DH 与控制器 TCP 的差（已测，约 1.5 mm / 0.2°） | 位置/姿态地板 |
| **real 对 real 重复性** | E2 中同工况 5 次重复之间的差 | 每格的重复误差 = 最关键的地板 |

产出合成 `calib/budget.json`，Gate B 用它判断差距是否显著。

#### E1 辨识集（训练数据，新增，约 20 分钟）

参照 Weigand 和 PACE：**六个关节都要激励**，覆盖不同速度和加速度，不受评测工况限制。
- 每个关节的 chirp（0.05–2 Hz 扫频）+ 多关节同时运动的 Fourier 激励轨迹，各 2–3 分钟；
- 在两个负载下各做一遍（裸载具，以及加一个已知质量块，例如 0.5 kg），让负载成为可学习的变量；
- **全部公开**，作为用户辨识仿真参数或训练修正模型的训练数据。

> 为什么需要：现在的 54 格只动肘关节，是**评测**集。没有独立的训练集，用户只能在评测数据上调参，
> 那就成了作弊。Weigand 的基准也是先给辨识数据，再在独立轨迹上评分。

#### E2 核心评测：54 工况 × 5 次重复（已实现，约 52 分钟机械臂时间）

- 6 档肘关节速度（0.2–0.9 rad/s）× 3 种臂型 × 3 种轨迹类型 = 54 格，每格 5 次，共 270 次；
- 3 个 session，隔天进行；第 3 个 session 前做一次刻意的载具重装，以测量重装误差；
- 54 格中有 18 格整格留出，固定种子，并按速度分层；
- 每次同时记录 125 Hz 的 RTDE 全字段（43 个，含**关节电流**和目标力矩）、IMU，以及按基准格式采样的运行文件。

**建议补充记录关节电流的差距：** 电流/力矩对负载、摩擦和惯量的差异最敏感，
比 TCP 位置更早暴露动力学差距。RTDE 已经在记录这些字段，只需在评分里加一个通道。

#### E3 分布外测试（新增，约 15 分钟）

参照 Lutter 的结论：模型真正的考验在分布外。
- **任务分布外：** arc_scan 检测路径（多关节、连续姿态变化）× 5 次。这也是检测这一应用场景本身；
- **负载分布外：** E2 中挑 6 格，在 E1 没见过的负载下各做 3 次；
- 这一组**全部不公开**，只用于打分。

#### E4 仿真侧：生成可比较的 sim 数据（在电脑上完成）

对 E1–E3 的每次运行，用同一个 `target_q` 在 MuJoCo 中回放（`sim_mujoco.py`，已实现），生成四个仿真版本，
它们同时也是赛道 A 的基线行：

| 版本 | 说明 | 预期 |
|---|---|---|
| S0 默认 | menagerie 原样（伺服 kp 2000 / kv 400，滞后约 0.2 s） | 差距大，随速度增大 |
| S1 前馈 | 加入速度前馈，模拟真实控制器的跟踪方式 | 去掉大部分滞后 |
| S2 辨识 | 在 E1 上用 CMA-ES 辨识摩擦、armature、延迟、PD 参数（PACE 式） | 关节误差显著下降 |
| S3 Isaac（可选） | 同样的指令在 Isaac Sim 中回放 | 不同引擎的对比 |

赛道 B 的基线（修正模型）：B0 identity；B1 常数偏移；B2 线性/逐格偏移；B3 小型 GRU 残差模型（NAS 式）。

#### E5 验证基准本身（数据齐了之后）

| 检验 | 方法 | 通过标准 |
|---|---|---|
| Gate B 显著性 | 差距 / 地板 | ≥ 3 倍 |
| Gate C 有信号 | 差距在工况间的离散 vs 工况内重复 | 离散 ≥ 0.5 倍（已实现） |
| 区分度 | 对 S0–S2、B0–B3 做 bootstrap（按运行重采样）求 GCR 的 95% 置信区间 | 相邻基线的区间不重叠 |
| 预测性 | 对每格分别计算 sim 的误差排序和 real 的误差排序（例如超调、稳定时间随速度的趋势）的 SRCC 与 MMRV | SRCC 高、MMRV 低；S2 应优于 S0 |
| 稳定性 | 只用 session 1+2 与用全部 session 时，基线排名是否一致 | 排名不变 |

### 3.3 评分：成绩单包含什么

每次提交得到一张成绩单，**每个指标都与地板并列显示**：

| 通道 | 指标 | 单位 |
|---|---|---|
| 关节位置 | RMSE、p95 | deg |
| TCP 位置/姿态 | 中位数、p95、最大值 | mm / deg |
| 动态特征 | 超调、稳定时间、达速时间的误差 | % / ms |
| 关节电流/力矩（建议新增） | 归一化 RMSE | — |
| IMU 角速度 | 扣除延迟后的 p95（已实现） | rad/s |
| IMU 加速度 | 频谱差距（停止后的残余振动） | dB |
| 时间 | 互相关滞后 | ms |

**头条：** 留出工况上的 **GCR-p95**，其中 GCR = 1 − err(提交, real) / err(原始仿真, real)。
其次是分布外任务上的 GCR-p95，以及"相对地板的倍数"。

### 3.4 数据量够不够：怎么判断

- **评测侧（E2）：** 270 次运行是 Collins 那类研究的数量级。够不够由数据本身回答：
  session 1 完成后，计算每格中位差距的 bootstrap 置信区间。如果区间宽度小于工况间离散的 20%，
  5 次重复就够；否则把重复数增加到 8。
- **训练侧（E1）：** 20 分钟激励 × 两档负载，对 S2 这种几十个参数的辨识足够（PACE 类方法通常用几分钟的 chirp）。
  对 B3 这类学习模型可能偏少，这本身也是 benchmark 要测出来的东西：小数据下物理模型更稳（Lutter）。
- **合计机械臂时间约 1.5 小时**，分 3–4 个半天完成。

---

## 4. 现状与下一步

### 4.1 现在的数据够不够？

**不够，正式数据其实还没有开始采：**
- Session 1 进度是 0/108；
- 已有的 arc_scan 数据来自旧的 30003 连接，111 s 中有 93 s 没有机器人数据，只能用来验证工具链；
- IMU 安装校准还没做，预检里 FusionHub 当时也没有数据流进来。

已经准备好的部分：采集平台、批量执行器、三个臂型的示教、IMU 时间/安装校准工具、MuJoCo 回放、评分与 Gate B/C。

### 4.2 建议顺序

| 周 | 做什么 | 产出 |
|---|---|---|
| 第 1 周 | 让 FusionHub 稳定出数据 → E0（静止 1 h、翻转、**imu mount cal**）→ 跑 Session 1 | 地板、第一批 108 次运行 |
| 第 1 周（电脑上） | 用 Session 1 跑通 S0/S1 回放和打分，检查 Gate B/C 的初步结果和置信区间 | 判断重复数是否需要增加 |
| 第 2 周 | 实现并采集 E1 辨识集（两档负载）→ Session 2 | 训练集 |
| 第 3 周 | 载具重装 → 重新登记载具、重新 imu mount cal → Session 3 → E3 分布外 | 完整数据集 |
| 第 3–4 周 | S2 辨识、B1–B3 基线、E5 验证（bootstrap、SRCC/MMRV） | 第一版排行榜 |
| 第 5 周起 | 写论文：贡献 1 是基准（数据、协议、指标、验证），贡献 2 是基线模型，应用场景是检测 | 论文初稿 |

### 4.3 需要新开发的部分

1. **E1 激励任务**：多关节 chirp/Fourier，含安全范围检查，可复用现在的执行器和"按住移动"。
2. **评分新增通道**：关节空间误差、关节电流、动态特征（超调/稳定时间）、IMU 加速度频谱。
3. **MuJoCo 版本 S1（速度前馈）和 S2（CMA-ES 辨识）**。
4. **E5 验证脚本**：bootstrap 置信区间、SRCC、MMRV。
5. **提交接口**：赛道 A 接收 MJCF + 参数；赛道 B 接收一个 `predict(sim_run, command) -> real_run` 函数或输出文件。

### 4.4 需要你（和 Sam）决定的事

- E1 的第二档负载用多重的质量块，怎么固定在法兰上？
- 赛道 C（自带机器人）放在这篇论文里，还是作为后续工作？
- 仿真伺服滞后：作为"默认仿真"的差距保留在 S0，修正放在 S1。这样既保留了发现，也给出了修正方法。

---

## 参考文献

1. J. Collins, D. Howard, J. Leitner. *Quantifying the Reality Gap in Robotic Manipulation Tasks.* ICRA 2019. https://arxiv.org/abs/1811.01484
2. J. Collins, R. Brown, J. Leitner, D. Howard. *Traversing the Reality Gap via Simulator Tuning.* 2020. https://arxiv.org/abs/2003.01369
3. T. Erez, Y. Tassa, E. Todorov. *Simulation Tools for Model-Based Robotics: Comparison of Bullet, Havok, MuJoCo, ODE and PhysX.* ICRA 2015. https://homes.cs.washington.edu/~todorov/papers/ErezICRA15.pdf
4. J. Weigand et al. *Dataset and Baseline for an Industrial Robot Identification Benchmark.* 2022. https://kluedo.ub.rptu.de/frontdoor/index/index/docId/6731
5. A. Kadian et al. *Sim2Real Predictivity: Does Evaluation in Simulation Predict Real-World Performance?* RA-L 2020. https://arxiv.org/abs/1912.06321
6. X. Li et al. *Evaluating Real-World Robot Manipulation Policies in Simulation (SIMPLER).* CoRL 2024. https://proceedings.mlr.press/v270/li25c.html
7. X. Yang et al. (NVIDIA). *Robot Policy Evaluation for Sim-to-Real Transfer: A Benchmarking Perspective.* RSS 2025 Workshop. https://research.nvidia.com/publication/2025-06_robot-policy-evaluation-sim-real-transfer-benchmarking-perspective
8. *REALM: A Real-to-Sim Validated Benchmark for Generalization in Robotic Manipulation.* 2025. https://arxiv.org/abs/2512.19562
9. F. Golemo et al. *Sim-to-Real Transfer with Neural-Augmented Robot Simulation.* CoRL 2018. https://proceedings.mlr.press/v87/golemo18a.html
10. J. Hanna, P. Stone. *Grounded Action Transformation for Robot Learning in Simulation.* AAAI 2017. https://ojs.aaai.org/index.php/AAAI/article/view/11044
11. A. Zeng et al. *TossingBot: Learning to Throw Arbitrary Objects with Residual Physics.* RSS 2019. https://roboticsproceedings.org/rss15/p04.html
12. M. Lutter et al. *Differentiable Physics Models for Real-world Offline Model-based Reinforcement Learning.* ICRA 2021. https://arxiv.org/abs/2011.01734
13. S. Bauer et al. *Real Robot Challenge: A Robotics Competition in the Cloud.* NeurIPS 2021 Competition Track (PMLR 2022). https://proceedings.mlr.press/v176/bauer22a.html
14. E. Clochiatti, L. Scalera, P. Boscariol, A. Gasparetto. *Electro-mechanical modeling and identification of the UR5 e-series robot.* Robotica 42 (2024) 2430–2452.
15. *Emergent Dexterity via Diverse Resets and Large-Scale Reinforcement Learning.* 2026（UR7e 上使用 PACE 辨识：约 7° → <2° 关节 RMSE）。https://arxiv.org/abs/2603.15789
16. *Towards bridging the gap: Systematic sim-to-real transfer for diverse legged robots*（PACE）。2025. https://arxiv.org/abs/2509.06342
17. W. Zhao, J. P. Queralta, T. Westerlund. *Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey.* 2020. https://arxiv.org/abs/2009.13303
