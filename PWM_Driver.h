/**
 * @file PWM_Driver.h
 * @brief TMS320F28335 ePWM 生产级驱动接口
 * @author 底层驱动架构师
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include "DSP2833x_Device.h"     // TI 标准寄存器结构体
#include "DSP2833x_Examples.h"

/** @brief 系统主频假设为 150MHz */
#define CPU_FREQ_HZ         150000000L
#define LSPCLK_FREQ_HZ      (CPU_FREQ_HZ/4)

/** @brief 计数模式枚举 */
typedef enum {
	UP_COUNT = 0, /**< 向上计数（边沿对齐） */
	DOWN_COUNT = 1, /**< 向下计数 */
	UP_DOWN_COUNT = 2 /**< 增减计数（中心对齐） */
} CountMode_t;

/** @brief 停止模式枚举 */
typedef enum {
	FORCE_LOW = 0, /**< 强制拉低 */
	FORCE_HIGH = 1, /**< 强制拉高 */
	HI_Z = 2 /**< 高阻态 */
} StopMode_t;

/** @brief 故障触发源 */
typedef enum {
	TZ_SRC_OSHT1 = 0x0001, /**< 一次性故障引脚 TZ1 */
	TZ_SRC_OSHT2 = 0x0002 /**< 一次性故障引脚 TZ2 */
} TzSource_t;

/**
 * @class EPwmChannel
 * @brief ePWM 模块封装类
 */
class EPwmChannel {
public:
	/**
	 * @brief 构造函数
	 * @param regs 寄存器基地址 (如 &EPwm1Regs)
	 * @param id 模块 ID (1-6)
	 */
	EPwmChannel(volatile struct EPWM_REGS *regs, Uint16 id);

	/** @brief 硬件引脚初始化 (寄存器: GPAMUX, GPAPUD) */
	void bindGpio();

	/** @brief 初始化时基与模式 (寄存器: TBPRD, TBCTL) */
	void init(Uint32 freqHz, CountMode_t mode);

	/** @brief 动态设置占空比 (寄存器: CMPA, CMPB) */
	void setDuty(float dutyA, float dutyB);

	/** @brief 设置互补死区 (寄存器: DBRED, DBFED, DBCTL) */
	void setDeadband(Uint16 redNs, Uint16 fedNs);

	/** @brief 设置相位偏移 (寄存器: TBPHS, TBCTL) */
	void setPhase(float degrees);

	/** @brief 软件紧急强制拉低 (寄存器: TZFRC) */
	void forceOutputLow();

	/** @brief 配置硬件 TripZone 保护 (寄存器: TZSEL, TZCTL) */
	void configureTripZone(TzSource_t src, StopMode_t action);

	/** @brief 启动计数 (寄存器: TBCTL) */
	void start();

	/** @brief 停止计数 */
	void stop();
	/**
	 * @brief 动态调频（保持当前占空比百分比恒定）
	 * @param newFreq 新的频率 (Hz)
	 */
	void updateFrequency(Uint32 newFreq);
	/**
	 * @brief 一次性原子化更新频率和占空比
	 * @param freq 频率
	 * @param dutyA A通道占空比 (0.0~1.0)
	 * @param dutyB B通道占空比 (0.0~1.0)
	 */
	void updateParams(Uint32 freq, float dutyA, float dutyB);

private:
	volatile struct EPWM_REGS *_regs;
	Uint16 _id;
	Uint32 _tbclk; /**< 当前计算出的 TBCLK 频率 */
	Uint16 _period; /**< 当前 TBPRD 值 */
	float _currentDutyA;
	float _currentDutyB;
	CountMode_t _mode;

	/** @brief 内部方法：根据频率自动计算分频比 */
	void calcClockPrescaler(Uint32 freqHz, Uint16 *clkDiv, Uint16 *hspClkDiv);
	Uint16 calculatePRD(Uint32 freq);
	Uint16 calculateCMP(Uint16 prd, float duty);
};

#endif
