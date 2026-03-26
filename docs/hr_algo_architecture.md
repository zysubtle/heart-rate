# 智能手表运动心率算法 — 架构总纲

> 本文档为架构冻结文档。标注为"强冻结"的内容，后续模块开发不得随意修改；
> 若确需变更，须发起专项 review 并说明原因、影响面、替代方案。

---

## 1. 系统概述

### 1.1 目标

在腕式智能手表（Cortex-M4F / Apollo 3.5 级别 MCU）上实现运动心率算法。

### 1.2 输入

| 信号 | 通道数 | 采样率 | 数据类型 |
|------|--------|--------|----------|
| ACC  | 3 轴   | 25 Hz  | int16_t  |
| PPG  | 4 路   | 25 Hz  | int32_t  |

### 1.3 输出

| 字段 | 类型 | 说明 |
|------|------|------|
| hr_bpm | float | 最终心率 (BPM)；0.0f = 无有效输出 |
| confidence | uint8_t | 输出置信度 [0..100] |
| hr_state | hr_state_t | 心率状态机状态 |
| motion_state | motion_state_t | 运动状态 |
| main_ch | uint8_t | 主 PPG 通道 [0..3] |
| sqi_main | float | 主通道 SQI [0.0, 1.0] |

### 1.4 运行方式

- 8 秒滑动窗口（200 点 @ 25 Hz）
- 每秒更新一次，调用 `hr_algo_process_1s()`

### 1.5 工程约束

- 禁止动态内存分配（malloc / calloc / new）
- 禁止将大数组放在栈上；大缓冲区须放入 ctx 结构内
- 禁止引入大型第三方依赖
- 参数不得散落在各 .c 文件中，须集中管理

---

## 2. 模块划分 【强冻结】

### 2.1 模块列表

| 模块 | 文件 | 职责 |
|------|------|------|
| M1 采样与缓存 | `src/hr_sampling.c` + `src/hr_sampling.h` | 管理 ACC/PPG ring buffer，导出窗口数据 |
| M2 预处理 | `src/hr_preproc.c` + `src/hr_preproc.h` | ACC/PPG 滤波、去直流 |
| M3 SQI 与通道选择 | `src/hr_sqi.c` + `src/hr_sqi.h` | 计算各通道 SQI，选择 main_ch / backup_ch |
| M4 运动状态识别 | `src/hr_motion.c` + `src/hr_motion.h` | 基于 ACC 特征判断 motion_state |
| M5 运动伪影抑制 | `src/hr_mac.c` + `src/hr_mac.h` | 对 main_ch PPG 做自适应滤波 |
| M6 心率候选估计 | `src/hr_candidate.c` + `src/hr_candidate.h` | 在 raw/mac 双分支上运行 peak/FFT/ACF/pred |
| M7 融合与平滑 | `src/hr_fusion.c` + `src/hr_fusion.h` | 候选评分、排序、融合、平滑输出 |
| M8 状态机 | `src/hr_statemachine.c` + `src/hr_statemachine.h` | 管理 hr_state 转移 |
| M9 主流程集成 | `src/hr_algo.c` | 编排 M1~M8 + confidence 计算 + debug frame |

### 2.2 公共头文件

| 文件 | 内容 | 可见性 |
|------|------|--------|
| `include/hr_algo_types.h` | 核心枚举与结构 | Public |
| `include/hr_algo_api.h` | 对外 API 声明 | Public |
| `include/hr_algo_params.h` | 参数统一入口 | Public |
| `include/hr_algo_debug.h` | 调试日志结构 | Public |

### 2.2.1 内部共享头文件

| 文件 | 内容 | 可见性 |
|------|------|--------|
| `src/hr_algo_internal.h` | `hr_algo_ctx_t` 完整定义（嵌入各模块子上下文） | Internal (仅 `src/` 内部) |

`hr_algo_ctx_t` 在 `include/hr_algo_api.h` 中为前向声明（opaque），
完整定义仅在 `src/hr_algo_internal.h` 中，外部调用者不可见。
各模块实现时在此文件尾部追加自身子上下文字段。

### 2.3 可见性规则

- 外部调用者只需包含 `include/hr_algo_api.h`
- `src/hr_*.h` 为 internal API，仅在 `src/` 内部使用，不对外暴露
- 模块之间禁止通过 private 结构横向耦合；模块间通信只能通过 ctx 中的公共字段或显式函数参数

---

## 3. 核心类型定义 【强冻结】

### 3.1 hr_state_t

```c
typedef enum {
    HR_STATE_INIT       = 0,
    HR_STATE_ACQUIRE    = 1,
    HR_STATE_TRACK      = 2,
    HR_STATE_HOLDOVER   = 3,
    HR_STATE_REACQUIRE  = 4
} hr_state_t;
```

| 状态 | 含义 |
|------|------|
| INIT | 初始化完成，尚未开始采集 |
| ACQUIRE | 数据积累中，尚未产出稳定心率 |
| TRACK | 正常跟踪，输出可信 |
| HOLDOVER | 信号丢失或候选无效，使用历史值保持输出 |
| REACQUIRE | 信号恢复中，重新锁定 |

### 3.2 motion_state_t

```c
typedef enum {
    MOTION_STATE_REST      = 0,
    MOTION_STATE_WALK      = 1,
    MOTION_STATE_RUN       = 2,
    MOTION_STATE_IRREGULAR = 3
} motion_state_t;
```

IRREGULAR 表示"非周期、非稳定节律、无法按 WALK/RUN 建模的活动状态"。

### 3.3 hr_source_t

```c
typedef enum {
    HR_SOURCE_NONE      = 0,
    HR_SOURCE_PEAK_RAW  = 1,
    HR_SOURCE_PEAK_MAC  = 2,
    HR_SOURCE_FFT_RAW   = 3,
    HR_SOURCE_FFT_MAC   = 4,
    HR_SOURCE_ACF_RAW   = 5,
    HR_SOURCE_ACF_MAC   = 6,
    HR_SOURCE_PRED      = 7
} hr_source_t;
```

### 3.4 hr_candidate_t

```c
typedef struct {
    bool          valid;
    float         bpm;
    float         score;
    hr_source_t   source;
} hr_candidate_t;
```

**语义约定：**

- `valid`：true 表示有效候选
- `bpm`：心率估计值；无效时必须为 0.0f
- `score`：候选内部排序/可信度分数 [0.0, 1.0]，**不等于**最终输出 confidence；无效时必须为 0.0f
- `source`：候选来源方法

### 3.5 hr_output_t

```c
typedef struct {
    float            hr_bpm;
    uint8_t          confidence;
    hr_state_t       hr_state;
    motion_state_t   motion_state;
    uint8_t          main_ch;
    uint8_t          backup_ch;
    float            sqi_main;
} hr_output_t;
```

**取值约定：**

| 字段 | 类型 | 范围 | 无效值 |
|------|------|------|--------|
| hr_bpm | float | 40.0 ~ 220.0 | 0.0f |
| confidence | uint8_t | 0 ~ 100 | 0 |
| main_ch | uint8_t | 0 ~ 3 | — |
| backup_ch | uint8_t | 0 ~ 3 | 与 main_ch 相同表示无独立备选 |
| sqi_main | float | 0.0 ~ 1.0 | 0.0f |

### 3.6 score 与 confidence 的语义分离 【强冻结】

| 属性 | score | confidence |
|------|-------|------------|
| 所属 | hr_candidate_t | hr_output_t |
| 量纲 | float [0.0, 1.0] | uint8_t [0, 100] |
| 用途 | 候选之间的内部排序 | 对外输出的最终置信度 |
| 计算时机 | 模块 6 (候选估计) | 步骤 10 (confidence 计算) |

二者**禁止混用**。

---

## 4. 公共 API 【强冻结】

```c
hr_params_t            hr_algo_default_params(void);
void                   hr_algo_init(hr_algo_ctx_t *ctx, const hr_params_t *params);
void                   hr_algo_feed_acc(hr_algo_ctx_t *ctx, int16_t ax, int16_t ay, int16_t az);
void                   hr_algo_feed_ppg(hr_algo_ctx_t *ctx, int32_t ch0, int32_t ch1, int32_t ch2, int32_t ch3);
void                   hr_algo_process_1s(hr_algo_ctx_t *ctx);
const hr_output_t     *hr_algo_get_output(const hr_algo_ctx_t *ctx);
const hr_debug_frame_t*hr_algo_get_debug(const hr_algo_ctx_t *ctx);
```

`hr_algo_ctx_t` 为前向声明，完整定义仅在 `src/` 内部可见。

---

## 5. 主流程顺序 【强冻结】

`hr_algo_process_1s(ctx)` 内部严格按以下 12 步执行：

| 步骤 | 操作 | 输入 | 输出 | 失败降级 |
|------|------|------|------|----------|
| 1 | 数据完整性检查 | ring buffer 状态 | data_ready (bool) | 数据不足 → 跳到步骤 11，输出降级结果 |
| 2 | 导出最近 200 点 | acc_ring, ppg_ring | acc_raw[200][3], ppg_raw[200][4] | — |
| 3 | 预处理 ACC/PPG | acc_raw, ppg_raw | acc_filt[200][3], ppg_filt[200][4] | — |
| 4 | SQI 计算与通道选择 | ppg_filt[200][4] | sqi[4], main_ch, backup_ch | — |
| 5 | 运动状态识别 | acc_filt[200][3] | motion_state | — |
| 6 | MAC | ppg_filt[main_ch], acc_filt, motion_state | ppg_mac[200]；保留 raw 分支 | MAC 发散 → 仅依赖 raw 分支 |
| 7 | 心率候选估计 | raw 分支, mac 分支, motion_state, 历史 | candidates[7] | 部分方法无效 → 对应 candidate.valid=false |
| 8 | 状态机更新 | candidates, sqi, motion_state, 历史 | hr_state | — |
| 9 | 融合候选+平滑 | candidates, hr_state, motion_state, 历史 | hr_bpm | — |
| 10 | 计算 confidence | hr_state, sqi, 候选一致性, motion_state | confidence | — |
| 11 | 生成 hr_output_t | 所有结果 | output 结构 | — |
| 12 | 记录 hr_debug_frame_t | 所有中间结果 | debug frame | — |

### 5.1 关键因果约束

- **步骤 8（状态机）必须在步骤 10（confidence）之前执行**
- confidence 可读取当前 hr_state，但 hr_state 的转移决策禁止使用当前周期的 confidence
- 状态机输入仅限于：候选质量（valid/score）、SQI、motion_state、历史输出、历史状态

### 5.2 raw/mac 双分支路径

```
模块 2 输出 ──→ 模块 5 ──┬─ raw 分支 (ppg_filt[main_ch]) ──┬──→ 模块 6 → candidates
                        └─ mac 分支 (ppg_mac)             ─┘
```

- 模块 5 输入 raw，输出 mac；raw 本身不被修改，直接透传
- 模块 6 对 raw 和 mac 分别运行 peak / FFT / ACF → 产出 6 个候选
- pred candidate 不区分 raw/mac，来自历史跟踪，独立产出 → 第 7 个候选
- 模块 7 从 7 个候选中选择最优
- debug frame 记录全部 7 个候选

---

## 6. 模块依赖关系 【强冻结】

### 6.1 依赖矩阵

| 模块 | 依赖的上游模块 | 产出给下游模块 |
|------|--------------|--------------|
| M1 采样 | 外部 feed 调用 | M2 |
| M2 预处理 | M1 | M3, M4, M5 |
| M3 SQI | M2 | M5, M6, M7, M8 |
| M4 运动 | M2 | M5, M6, M7, M8 |
| M5 MAC | M2, M3, M4 | M6 |
| M6 候选 | M5(raw+mac), M3(sqi), M4(motion) | M7, M8 |
| M7 融合 | M6, M8, M4 | M9 |
| M8 状态机 | M6, M3, M4 | M7, M9 |
| M9 主流程 | M7, M8（编排所有模块） | 外部 |

### 6.2 共享字段唯一写入者

| 共享字段 | 唯一写入模块 | 允许读取的模块 | 禁止写入 |
|----------|-------------|---------------|---------|
| main_ch, backup_ch | M3 (SQI) | M5, M6, M7, M9 | 其他所有 |
| motion_state | M4 (Motion) | M5, M6, M7, M8, M9 | 其他所有 |
| hr_state | M8 (StateMachine) | M7, M9 | 其他所有 |
| candidates[7] | M6 (Candidate) | M7, M8 | 其他所有 |
| hr_bpm（最终） | M7 (Fusion) | M9 | 其他所有 |
| confidence | M9 (confidence 计算步骤) | 无（终端输出） | M8 禁止读取当前周期值 |
| sqi[4] | M3 (SQI) | M5, M6, M7, M8, M9 | 其他所有 |

---

## 7. 状态机职责 【强冻结】

### 7.1 hr_state 状态转移

```
           ┌────────────────────────┐
           ▼                        │
  INIT ──→ ACQUIRE ──→ TRACK ──→ HOLDOVER
                         ▲          │
                         │          ▼
                         └── REACQUIRE
```

- INIT → ACQUIRE：首次收到足够数据
- ACQUIRE → TRACK：连续 N 个周期产出稳定候选
- TRACK → HOLDOVER：候选质量持续恶化或全部无效
- HOLDOVER → REACQUIRE：HOLDOVER 超时或检测到信号恢复
- REACQUIRE → TRACK：重新锁定成功

### 7.2 motion_state 写入规则

- 由 M4（运动状态识别）负责写入
- 其他模块只读

### 7.3 禁止事项

- hr_state 的转移决策禁止依赖当前周期 confidence
- confidence 在状态机之后计算（步骤 10），可受当前 hr_state 影响，但反向不成立

---

## 8. 异常/降级场景输出契约 【强冻结】

| 场景 | hr_state | hr_bpm | confidence | debug_frame |
|------|----------|--------|------------|-------------|
| 冷启动 | INIT | 0.0 | 0 | 记录（标记 INIT） |
| ring buffer 未满 | ACQUIRE | 0.0 | 0 | 记录（标记数据不足） |
| 数据断流 | HOLDOVER | 历史值 | 逐周期递减至 0 | 记录（标记断流） |
| 全部 candidate 无效 | HOLDOVER / REACQUIRE | 历史值 | 逐周期递减 | 记录（标记无有效候选） |
| main_ch 频繁切换 | 不变 | 正常流程 | 可能降低 | 记录切换事件 |
| MAC 失败/发散 | 不变 | 仅依赖 raw 分支候选 | 可能降低 | 记录 MAC 降级 |

**降级规则：**

- pred candidate 作为保底候选，来源仅限上一周期稳定输出 / tracker 历史记忆
- pred candidate 不得为未定义常数或硬编码默认值
- HOLDOVER 状态输出上一次 TRACK 的 hr_bpm，confidence 逐周期递减
- confidence 递减到 0 且仍无有效候选 → REACQUIRE
- INIT / ACQUIRE 状态下 hr_bpm = 0.0，confidence = 0
- 任何场景下 debug frame 必须记录，不得跳过

---

## 9. 参数管理 【强冻结：统一入口原则 / 中冻结：具体字段】

### 9.1 原则

- 所有可调参数集中在 `hr_params_t` 结构中
- 各模块通过 ctx 引用参数，禁止在 .c 中硬编码
- `hr_algo_default_params()` 提供编译期默认值
- 调用者可修改后传入 `hr_algo_init()`

### 9.2 参数分级

| 级别 | 说明 | 示例 |
|------|------|------|
| 编译期默认 | 由 default_params() 返回 | 所有参数 |
| 运行时可覆盖 | init 时传入 | 所有参数 |
| 重点调参对象 | 后续优化重点 | nlms_step_size, smooth_alpha, sqi 门限, hr_min/max_bpm, acquire/holdover count |

### 9.3 子结构划分

参数按模块分组为嵌套子结构：preproc / sqi / motion / mac / candidate / fusion / statemachine。
具体字段详见 `include/hr_algo_params.h`。

---

## 10. 调试日志 【强冻结：主框架 / 中冻结：扩展字段】

### 10.1 hr_debug_frame_t 主框架

每个周期必须产出一帧，包含：

- 时间戳 (timestamp_ms)
- 状态 (hr_state, motion_state)
- 通道 (main_ch, backup_ch, sqi[4])
- 7 个候选详情 (cand_peak_raw/mac, cand_fft_raw/mac, cand_acf_raw/mac, cand_pred)
- 最终输出 (hr_out, confidence)
- 异常标志 (flags)

### 10.2 候选日志字段

每个 candidate 在日志中至少记录：
- valid (bool)
- bpm (float)
- score (float [0.0, 1.0])
- source (hr_source_t)

无效 candidate 表示为：valid=false, bpm=0.0f, score=0.0f。

### 10.3 flags 位域

预留 uint32_t，具体 bit 分配为中冻结，在各模块实现时逐步确定。

### 10.4 日志输出机制

日志输出传输机制（串口 / SPI Flash / RAM buffer）为待定项，不在本轮冻结范围。

---

## 11. Internal API 签名 【中冻结】

| 模块 | 初始化 | 处理 | 职责 |
|------|--------|------|------|
| M1 | hr_sampling_init() | hr_sampling_push_acc/ppg(), hr_sampling_export() | ring buffer 管理 |
| M2 | hr_preproc_init() | hr_preproc_run() | 滤波、去直流 |
| M3 | hr_sqi_init() | hr_sqi_run() | SQI 计算、通道选择 |
| M4 | hr_motion_init() | hr_motion_run() | 运动状态判断 |
| M5 | hr_mac_init() | hr_mac_run() | 自适应滤波 |
| M6 | hr_candidate_init() | hr_candidate_run() | peak/FFT/ACF/pred 候选生成 |
| M7 | hr_fusion_init() | hr_fusion_run() | 候选融合、平滑 |
| M8 | hr_sm_init() | hr_sm_update() | 状态转移 |

具体参数签名在实现时允许小范围调整，但职责边界不得变。

---

## 12. 高风险点与禁止事项

### 12.1 高风险点

1. raw/mac 双分支容易退化为只用其中一个
2. score 与 confidence 容易混淆
3. hr_state 与 confidence 容易形成循环依赖
4. 参数容易散落到各 .c 中硬编码
5. 主流程容易膨胀为巨型函数
6. pred candidate 容易退化为硬编码兜底值
7. 200x4 float 数组（约 3.2KB）不可放栈上

### 12.2 禁止事项

- 禁止在 .c 中硬编码可调参数
- 禁止越权写入非自身负责的共享字段
- 禁止 hr_state 转移逻辑读取当前周期 confidence
- 禁止使用 malloc/calloc/new
- 禁止在主流程中嵌入具体算法逻辑
- 禁止跳过 debug frame 记录
- 禁止将 internal header 暴露到 include/
- 禁止单个候选同时兼任排序分数和最终置信度
- 禁止 pred candidate 使用硬编码常数作为 bpm 来源

### 12.3 最小修正规则

1. 强冻结项：须专项 review，说明原因、影响面、替代方案
2. 中冻结项：可在模块 PR 中附带说明，不得破坏顶层语义
3. 新增字段优先加在结构尾部
4. 枚举值只可追加，不可重排

---

## 13. 未决项清单

| # | 问题 | 推荐默认方案 | 定稿时机 |
|---|------|-------------|----------|
| O1 | FFT 长度（200 vs 256 补零） | **已定稿 (M6)**：Goertzel selective DFT，200 点数据 + 256 点隐式零填充 | ~~M6 前~~ 已实现 |
| O2 | SQI 子指标拆分 | **已定稿 (M3)**：periodicity (ACF) + peak_regularity (inter-peak CV)，权重 0.6/0.4 | ~~M3 前~~ 已实现 |
| O3 | confidence 计算公式 | **已定稿 (M9)**：TRACK 加权线性组合 (SQI 0.30 + score 0.35 + consistency 0.20 + motion 0.15)；HOLDOVER 每周期 -10；REACQUIRE 同 TRACK 但 cap 60 | ~~M7/M9 前~~ 已实现 |
| O4 | score 归一化方式 | **已定稿 (M6)**：各方法内部归一化到 [0,1]，含 SQI/motion 调制 | ~~M6 前~~ 已实现 |
| O5 | pred candidate 策略 | **已定稿 (M6+M7)**：M7 在 TRACK 稳定 3 周期后注入 pred seed；M6 读取 seed 生成 pred candidate | ~~M6 前~~ 已实现 |
| O6 | MAC 算法选择 | **已定稿 (M5)**：多输入 NLMS (3 轴动态 ACC 参考)，REST 旁路，发散检测+重置 | ~~M5 前~~ 已实现 |
| O7 | HOLDOVER confidence 递减策略 | **已定稿 (M9)**：每周期 -10，最低 0，基于 prev_confidence | ~~M8 前~~ 已实现 |
| O8 | 滤波器设计 | **已定稿 (M2)**：2 阶 IIR Butterworth (DF2T biquad)，block-stateless + DC warm-up | ~~M2 前~~ 已实现 |
| O9 | ACC 运动分类指标 | **已定稿 (M4)**：动态能量 + ACF 周期性 + 主频 (lag→Hz)，规则分类 | ~~M4 前~~ 已实现 |
| O10 | 日志输出机制 | RAM ring buffer | **未实现**，结构已冻结，传输层待集成阶段 |
| O11 | main_ch 切换迟滞 | **已定稿 (M3)**：switch_margin + switch_hold_count 参数化迟滞，已入 hr_params_t.sqi | ~~M3 前~~ 已实现 |
| O12 | 融合策略 | **已定稿 (M7)**：rank = W_SCORE * score + W_HIST * consistency，jump-limit + EMA 平滑 | ~~M7 前~~ 已实现 |
| O13 | flags 位域分配 | **首版定稿 (M9)**：8 bit 已分配 (DATA_NOT_READY/MAC_BYPASSED/MAC_DIVERGED/NO_VALID_CANDIDATE/HOLDOVER_ACTIVE/REACQUIRE_ACTIVE/MAIN_CH_SWITCHED/SIGNAL_RECOVERED) | ~~各模块实现时~~ V1 已分配 |
| O14 | 构建系统 | CMake | **未实现**，当前仅 tests/Makefile |

---

## 14. 后续开发顺序

```
M1 采样缓存
  │
  ▼
M2 预处理
  │
  ├──→ M3 SQI（可与 M4 并行）
  ├──→ M4 运动识别
  │
  ▼
【Review #1：验证 M1~M4 接口对齐、数据流通畅】
  │
  ▼
M5 MAC
  │
  ▼
M6 候选估计
  │
  ▼
【Review #2：验证 raw/mac 双分支、candidate 语义、score 量纲】
  │
  ▼
M8 状态机
  │
  ▼
M7 融合平滑
  │
  ▼
【Review #3：验证 state/confidence 无循环依赖、降级契约】
  │
  ▼
M9 主流程集成
  │
  ▼
【Review #4：全链路端到端验证、日志完整性、参数无散落】
```

---

## 15. 冻结清单汇总

### 强冻结

| 对象 | 文件 | 语义 | 允许修改范围 |
|------|------|------|-------------|
| hr_state_t | hr_algo_types.h | 五态枚举 | 可追加，不可删除/重排 |
| motion_state_t | hr_algo_types.h | 四态枚举 | 可追加，不可删除/重排 |
| hr_source_t | hr_algo_types.h | 7 源 + NONE | 可追加，不可删除/重排 |
| hr_candidate_t | hr_algo_types.h | valid/bpm/score/source | 可尾部追加，不可删改已有 |
| hr_output_t | hr_algo_types.h | 7 字段 | 可尾部追加，不可删改已有 |
| score vs confidence | 跨文件 | 独立量，禁止混用 | 不可合并 |
| 公共 API (7 函数) | hr_algo_api.h | 签名冻结 | 不可随意修改 |
| 主流程 12 步顺序 | hr_algo.c | 严格顺序 | 不可调换步骤 8 和 10 |
| hr_state 不依赖 confidence | 跨模块 | 因果约束 | 绝对不可违反 |
| 共享字段写入权 | 架构文档 | 唯一写入者表 | 不可变更 |
| hr_debug_frame_t 主框架 | hr_algo_debug.h | 7 候选+状态+输出+flags | 可尾部扩展，不可删减 |
| 9 模块独立文件 | 目录结构 | 每模块独立 | 不可随意合并 |
| 无效 candidate 表示 | hr_algo_types.h | valid=false,bpm=0,score=0 | 不可变更 |
| 无效 hr_bpm | hr_algo_types.h | 0.0f = 无效 | 不可变更 |
| 禁止动态内存 | 全局 | 无 malloc/new | 除非统一内存池 |
| pred candidate 来源 | 架构文档 | 历史稳定输出 | 不可放宽 |

### 中冻结

| 对象 | 语义 | 允许修改范围 |
|------|------|-------------|
| hr_params_t 子字段 | 按模块分组参数 | 可增删字段，不可打破统一入口 |
| hr_algo_ctx_t 内部 | 前向声明 | 可自由调整，不可暴露为 public |
| 各模块 internal API | 见第 11 节 | 参数可调，职责不可变 |
| flags 位域 | uint32_t 预留 | 实现时分配 |
| backup_ch 无备选编码 | 与 main_ch 相同 | 可调整编码方式 |

### 待定项

见第 13 节（O1~O14）。
