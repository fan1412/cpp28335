# TMS320F28335 ePWM C++ 驱动功能差距分析

基于 `PWM_Driver.h/.cpp` 当前实现，建议按“必须补齐（安全/正确性）→增强可用性→工程化”三个层级补充功能。

## 1) 必须补齐（优先级 P0）

1. **输入参数与边界保护**
   - `updateParams()` / `updateFrequency()` 目前未做 `freq==0` 防护，`calculatePRD()` 直接除法存在除零风险。
   - `dutyA/dutyB` 仅在 `setDuty()` 中限幅，`updateParams()` 未限幅会导致 CMP 溢出或反向。
   - 建议新增统一的 `sanitizeParams()`，对频率范围、占空比范围、PRD 上下限进行约束并返回状态码。

2. **频率更新后内部状态一致性**
   - `updateFrequency()` / `updateParams()` 写入了 `TBPRD`，但没有回写 `_period`，导致后续 `setDuty()` 仍使用旧周期计算 CMP。
   - 建议在所有改频路径中同步维护 `_period` 与 `_tbclk`。

3. **时钟分频策略真正落地**
   - 已有 `calcClockPrescaler()`，但 `init()` 中固定 `_tbclk = 75MHz`，未根据目标频率写 `TBCTL.CLKDIV/HSPCLKDIV`。
   - 建议新增 `setTimebaseClock(freq)`：自动选分频 + 写寄存器 + 计算真实 TBCLK，保证低频场景仍具备分辨率。

4. **Trip Zone 完整闭环**
   - 目前可配置/触发故障，但缺少 `clearTripFlags()` 与故障状态读取（`TZFLG`）接口。
   - 建议新增：`getTripStatus()`、`clearTrip(ost/cbc)`、`registerFaultCallback()`（可选）。

5. **启动/停止与同步行为可控**
   - 当前 `start()` 仅恢复 CTRMODE，未处理 `TBCLKSYNC` 全局同步与软件同步 `SWFSYNC`。
   - 建议提供：`enableGlobalSync()`、`softwareSync()`、`startSynced()`，并在文档中明确主从 ePWM 使用方式。

## 2) 可用性增强（优先级 P1）

1. **AQ 动作配置可参数化**
   - 当前 AQ 逻辑写死，且主要覆盖 A 通道。
   - 建议开放 `configureActionQualifier(channel, event, action)`，并补齐 B 通道（`AQCTLB`）。

2. **相位控制接口落地**
   - 头文件声明了 `setPhase(float degrees)`，实现文件中尚未提供定义。
   - 建议按 `TBPHS` + `PHSEN` + `SYNCOSEL` 完整实现，并支持度数到计数值换算。

3. **死区能力扩展**
   - 目前仅支持 ns 配置与固定极性（AHC）。
   - 建议支持：独立输入源（A/B）、极性模式选择、按 TBCLK tick 直接配置接口。

4. **计数器/波形调试接口**
   - 增加只读 API：`getCounter()`、`getPeriod()`、`getCmpA/B()`，方便在线诊断与自检。

5. **统一错误码与返回值**
   - 当前接口基本为 `void`，调用方无法判断参数非法、资源冲突、状态异常。
   - 建议引入 `enum class PwmStatus`，关键配置接口返回状态。

## 3) 工程化能力（优先级 P2）

1. **中断事件支持**
   - 补充 ePWM 中断配置：`ETSEL/ETPS/ETCLR`，支持 CTR=0/PRD/CMP 触发与回调注册。

2. **ADC SOC 触发支持**
   - 增加 `configureAdcSocA/B()`，让 PWM 在固定相位触发采样，适配电机/电源闭环。

3. **Chopper 与 HRPWM（按芯片能力）**
   - 若项目使用相关特性，建议封装 `PCCTL`（斩波）及高分辨率扩展。

4. **多通道管理器**
   - 引入 `EPwmManager` 负责：模块依赖关系、同步链、批量原子更新。

5. **最小化单元测试/仿真桩**
   - 为 `calculatePRD/calculateCMP`、参数限幅、分频选择建立主机侧测试，降低回归风险。

## 4) 建议优先实现顺序（落地路线）

1. P0：参数校验 + `_period` 状态修复 + `setPhase()` 补齐。
2. P0：时钟分频真正接入 `TBCTL`，并验证高/低频边界。
3. P0：Trip Zone 查询/清除接口，形成保护闭环。
4. P1：AQ/B 通道配置化 + 错误码体系。
5. P2：中断、ADC 触发、多通道管理与测试基础设施。

## 5) 对当前代码的两个关键风险提醒

- **风险 1：状态不一致**
  频率更新后 `_period` 未同步，后续 `setDuty()` 结果可能与期望不符。
- **风险 2：接口声明与实现不一致**
  `setPhase()` 已声明但未实现，链接阶段会报 undefined reference（一旦调用）。
