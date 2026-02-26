/**
 * @file PWM_Driver.cpp
 * @brief TMS320F28335 ePWM 驱动实现
 */

#include "PWM_Driver.h"

EPwmChannel::EPwmChannel(volatile struct EPWM_REGS *regs, Uint16 id) :
		_regs(regs), _id(id), _tbclk(CPU_FREQ_HZ), _period(0), _mode(UP_DOWN_COUNT) {
}

void EPwmChannel::bindGpio() {
	EALLOW;
	switch (_id) {
	case 1:
		InitEPwm1Gpio();
		break;
	case 2:
		InitEPwm2Gpio();
		break;
	case 3:
		InitEPwm3Gpio();
		break;
	case 4:
		InitEPwm4Gpio();
		break;
	case 5:
		InitEPwm5Gpio();
		break;
	case 6:
		InitEPwm6Gpio();
		break;
	}
	EDIS;
}

void EPwmChannel::init(Uint32 freqHz, CountMode_t mode) {
	_mode = mode;
	_tbclk = 75000000; // 假设系统配置为 150MHz/2 = 75MHz

	EALLOW;
// 1. 配置影子寄存器加载
	_regs->TBCTL.bit.PRDLD = TB_SHADOW;       // PRD 开启影子模式
	_regs->CMPCTL.bit.SHDWAMODE = CC_SHADOW;  // CMPA 开启影子模式
	_regs->CMPCTL.bit.SHDWBMODE = CC_SHADOW;

// 2. 配置加载时机 (CTR=0 时统一更新)
	_regs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	_regs->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

// 3. 配置计数模式
	_regs->TBCTL.bit.CTRMODE = (Uint16) mode;

// 4. 配置动作限定 (AQ) - 简化逻辑示例
	if (mode == UP_DOWN_COUNT) {
		_regs->AQCTLA.bit.CAU = AQ_SET;       // 递增时匹配置位
		_regs->AQCTLA.bit.CAD = AQ_CLEAR;     // 递减时匹配清零
	} else {
		_regs->AQCTLA.bit.ZRO = AQ_SET;       // 归零置位
		_regs->AQCTLA.bit.CAU = AQ_CLEAR;     // 匹配清零
	}
	EDIS;

// 设置初始频率，占空比设为 0
	updateParams(freqHz, 0.0f, 0.0f);}

void EPwmChannel::setDuty(float dutyA, float dutyB) {
	// 限幅检查
	if (dutyA > 1.0f)
		dutyA = 1.0f;
	else if (dutyA < 0.0f)
		dutyA = 0.0f;
	if (dutyB > 1.0f)
		dutyB = 1.0f;
	else if (dutyB < 0.0f)
		dutyB = 0.0f;

	// 转换 float 为 Uint16 计数值
	// 中心对齐模式下: CMPA = Period * (1 - Duty)
	Uint16 cmpA = (Uint16) ((float) _period * (1.0f - dutyA));
	Uint16 cmpB = (Uint16) ((float) _period * (1.0f - dutyB));

	_regs->CMPA.half.CMPA = cmpA;
	_regs->CMPB = cmpB;
}

void EPwmChannel::setDeadband(Uint16 redNs, Uint16 fedNs) {
	// 28335 死区模块使用 TBCLK
	// 计数值 = 时间 / (1 / TBCLK) = 时间 * TBCLK
	Uint16 redVal = (Uint16) ((float) redNs * _tbclk / 1e9);
	Uint16 fedVal = (Uint16) ((float) fedNs * _tbclk / 1e9);

	EALLOW;
	_regs->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // 开启上升/下降沿死区
	_regs->DBCTL.bit.POLSEL = DB_ACTV_HIC;      // 高电平有效互补 (AHC)
	_regs->DBCTL.bit.IN_MODE = DBA_ALL;         // EPWMxA 作为输入源

	_regs->DBRED = (redVal > 1023) ? 1023 : redVal; // 10位寄存器限幅
	_regs->DBFED = (fedVal > 1023) ? 1023 : fedVal;
	EDIS;
}

void EPwmChannel::configureTripZone(TzSource_t src, StopMode_t action) {
	EALLOW;
	// 配置触发源 (一次性触发)
	_regs->TZSEL.all |= (Uint16) src;

	// 配置触发后的动作
	if (action == FORCE_LOW) {
		_regs->TZCTL.bit.TZA = TZ_FORCE_LO;
		_regs->TZCTL.bit.TZB = TZ_FORCE_LO;
	} else if (action == HI_Z) {
		_regs->TZCTL.bit.TZA = TZ_HIZ;
		_regs->TZCTL.bit.TZB = TZ_HIZ;
	}
	EDIS;
}

void EPwmChannel::forceOutputLow() {
	EALLOW;
	_regs->TZFRC.bit.OST = 1; // 手动触发一次性故障封锁输出
	EDIS;
}
void EPwmChannel::start() {
	// 注意：实际应用中建议通过全局同步位控制，这里仅修改本地模式
	EALLOW;
	_regs->TBCTL.bit.CTRMODE = (Uint16) _mode;
	EDIS;
}

void EPwmChannel::stop() {
	EALLOW;
	_regs->TBCTL.bit.CTRMODE = TB_FREEZE;
	EDIS;
}

void EPwmChannel::updateFrequency(Uint32 newFreq) {
    // 频率变化时，必须重算 PRD 并按比例重算当前的 CMP
    Uint16 newPrd = calculatePRD(newFreq);
    Uint16 newCmpA = calculateCMP(newPrd, _currentDutyA);
    Uint16 newCmpB = calculateCMP(newPrd, _currentDutyB);

    // 顺序写入：由于启用了影子加载，这些值会暂存在缓冲区，直到 CTR=0
    _regs->TBPRD = newPrd;
    _regs->CMPA.half.CMPA = newCmpA;
    _regs->CMPB = newCmpB;
}

void EPwmChannel::updateParams(Uint32 freq, float dutyA, float dutyB) {
    _currentDutyA = dutyA;
    _currentDutyB = dutyB;

    Uint16 newPrd = calculatePRD(freq);
    Uint16 newCmpA = calculateCMP(newPrd, dutyA);
    Uint16 newCmpB = calculateCMP(newPrd, dutyB);

    // 原子同步更新
    _regs->TBPRD = newPrd;
    _regs->CMPA.half.CMPA = newCmpA;
    _regs->CMPB = newCmpB;
}

void EPwmChannel::calcClockPrescaler(Uint32 freqHz, Uint16 *clkDiv, Uint16 *hspClkDiv) {
	// 简单的分频自动选择逻辑：为了维持精度，优先减小分频
	// F28335 TBPRD 最大 65535
	if (freqHz > 5000) {
		*clkDiv = 0;     // /1
		*hspClkDiv = 1;  // /2 (HSPCLKDIV=1 实际代表 /2)
	} else if (freqHz > 1000) {
		*clkDiv = 1;     // /2
		*hspClkDiv = 2;  // /4
	} else {
		*clkDiv = 7;     // /128
		*hspClkDiv = 7;  // /14
	}
}

Uint16 EPwmChannel::calculatePRD(Uint32 freq) {
	if (_mode == UP_DOWN_COUNT) {
		// 公式: TBPRD = TBCLK / (2 * freq)
		return (Uint16) (_tbclk / (2 * freq));
	} else {
		// 公式: TBPRD = TBCLK / freq - 1
		return (Uint16) (_tbclk / freq - 1);
	}
}

// 私有方法：根据 PRD 自动计算 CMP 计数值
Uint16 EPwmChannel::calculateCMP(Uint16 prd, float duty) {
	if (_mode == UP_DOWN_COUNT) {
		// 中心对称模式：CMP = PRD * (1 - duty)
		return (Uint16) ((float) prd * (1.0f - duty));
	} else {
		// 向上计数模式：CMP = PRD * duty
		return (Uint16) ((float) prd * duty);
	}
}
