/**
 * @file PWM_Driver.cpp
 * @brief TMS320F28335 ePWM 驱动实现
 */

#include "PWM_Driver.h"

namespace {

Uint16 clampDuty(float duty) {
	if (duty < 0.0f) {
		return 0;
	}
	if (duty > 1.0f) {
		return 10000;
	}
	return (Uint16) (duty * 10000.0f);
}

float dutyFromQ(Uint16 dutyQ) {
	return (float) dutyQ / 10000.0f;
}

Uint16 decodeClkDiv(Uint16 clkDivBits) {
	return (Uint16) (1U << clkDivBits);
}

Uint16 decodeHspDiv(Uint16 hspDivBits) {
	static const Uint16 kHspDivTable[8] = { 1, 2, 4, 6, 8, 10, 12, 14 };
	return kHspDivTable[hspDivBits & 0x7U];
}

} // namespace

EPwmChannel::EPwmChannel(volatile struct EPWM_REGS *regs, Uint16 id) :
		_regs(regs), _id(id), _tbclk(CPU_FREQ_HZ), _period(0), _currentDutyA(0.0f), _currentDutyB(
				0.0f), _mode(UP_DOWN_COUNT) {
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
	Uint16 clkDiv = 0;
	Uint16 hspClkDiv = 0;

	_mode = mode;
	if (freqHz == 0U) {
		freqHz = 1U;
	}

	calcClockPrescaler(freqHz, &clkDiv, &hspClkDiv);
	_tbclk = CPU_FREQ_HZ / (decodeClkDiv(clkDiv) * decodeHspDiv(hspClkDiv));

	EALLOW;
	// 1. 配置时基时钟
	_regs->TBCTL.bit.CLKDIV = clkDiv;
	_regs->TBCTL.bit.HSPCLKDIV = hspClkDiv;

	// 2. 配置影子寄存器加载
	_regs->TBCTL.bit.PRDLD = TB_SHADOW;
	_regs->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	_regs->CMPCTL.bit.SHDWBMODE = CC_SHADOW;

	// 3. 配置加载时机 (CTR=0 时统一更新)
	_regs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	_regs->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	// 4. 配置计数模式
	_regs->TBCTL.bit.CTRMODE = (Uint16) mode;

	// 5. 配置动作限定 (AQ) - 简化逻辑示例
	if (mode == UP_DOWN_COUNT) {
		_regs->AQCTLA.bit.CAU = AQ_SET;
		_regs->AQCTLA.bit.CAD = AQ_CLEAR;
	} else {
		_regs->AQCTLA.bit.ZRO = AQ_SET;
		_regs->AQCTLA.bit.CAU = AQ_CLEAR;
	}
	EDIS;

	updateParams(freqHz, 0.0f, 0.0f);
}

void EPwmChannel::setDuty(float dutyA, float dutyB) {
	Uint16 dutyAQ = clampDuty(dutyA);
	Uint16 dutyBQ = clampDuty(dutyB);
	_currentDutyA = dutyFromQ(dutyAQ);
	_currentDutyB = dutyFromQ(dutyBQ);

	_regs->CMPA.half.CMPA = calculateCMP(_period, _currentDutyA);
	_regs->CMPB = calculateCMP(_period, _currentDutyB);
}

void EPwmChannel::setDeadband(Uint16 redNs, Uint16 fedNs) {
	Uint16 redVal = (Uint16) ((float) redNs * _tbclk / 1e9);
	Uint16 fedVal = (Uint16) ((float) fedNs * _tbclk / 1e9);

	EALLOW;
	_regs->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	_regs->DBCTL.bit.POLSEL = DB_ACTV_HIC;
	_regs->DBCTL.bit.IN_MODE = DBA_ALL;

	_regs->DBRED = (redVal > 1023U) ? 1023U : redVal;
	_regs->DBFED = (fedVal > 1023U) ? 1023U : fedVal;
	EDIS;
}

void EPwmChannel::setPhase(float degrees) {
	Uint32 cycleCounts;
	Uint32 phaseCount;

	while (degrees < 0.0f) {
		degrees += 360.0f;
	}
	while (degrees >= 360.0f) {
		degrees -= 360.0f;
	}

	if (_mode == UP_DOWN_COUNT) {
		cycleCounts = (Uint32) _period * 2U;
	} else {
		cycleCounts = (Uint32) _period + 1U;
	}
	if (cycleCounts == 0U) {
		cycleCounts = 1U;
	}

	phaseCount = (Uint32) ((degrees / 360.0f) * (float) cycleCounts);
	if (phaseCount > 0xFFFFU) {
		phaseCount = 0xFFFFU;
	}

	EALLOW;
	_regs->TBCTL.bit.PHSEN = TB_ENABLE;
	_regs->TBPHS.half.TBPHS = (Uint16) phaseCount;
	_regs->TBCTL.bit.SWFSYNC = 1;
	EDIS;
}

void EPwmChannel::configureTripZone(TzSource_t src, StopMode_t action) {
	EALLOW;
	_regs->TZSEL.all |= (Uint16) src;

	if (action == FORCE_LOW) {
		_regs->TZCTL.bit.TZA = TZ_FORCE_LO;
		_regs->TZCTL.bit.TZB = TZ_FORCE_LO;
	} else if (action == FORCE_HIGH) {
		_regs->TZCTL.bit.TZA = TZ_FORCE_HI;
		_regs->TZCTL.bit.TZB = TZ_FORCE_HI;
	} else {
		_regs->TZCTL.bit.TZA = TZ_HIZ;
		_regs->TZCTL.bit.TZB = TZ_HIZ;
	}
	EDIS;
}

void EPwmChannel::forceOutputLow() {
	EALLOW;
	_regs->TZFRC.bit.OST = 1;
	EDIS;
}

void EPwmChannel::start() {
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
	if (newFreq == 0U) {
		newFreq = 1U;
	}

	_period = calculatePRD(newFreq);
	_regs->TBPRD = _period;
	_regs->CMPA.half.CMPA = calculateCMP(_period, _currentDutyA);
	_regs->CMPB = calculateCMP(_period, _currentDutyB);
}

void EPwmChannel::updateParams(Uint32 freq, float dutyA, float dutyB) {
	if (freq == 0U) {
		freq = 1U;
	}

	_currentDutyA = dutyFromQ(clampDuty(dutyA));
	_currentDutyB = dutyFromQ(clampDuty(dutyB));
	_period = calculatePRD(freq);

	_regs->TBPRD = _period;
	_regs->CMPA.half.CMPA = calculateCMP(_period, _currentDutyA);
	_regs->CMPB = calculateCMP(_period, _currentDutyB);
}

void EPwmChannel::calcClockPrescaler(Uint32 freqHz, Uint16 *clkDiv, Uint16 *hspClkDiv) {
	if (freqHz > 5000U) {
		*clkDiv = 0;
		*hspClkDiv = 1;
	} else if (freqHz > 1000U) {
		*clkDiv = 1;
		*hspClkDiv = 2;
	} else {
		*clkDiv = 7;
		*hspClkDiv = 7;
	}
}

Uint16 EPwmChannel::calculatePRD(Uint32 freq) {
	Uint32 prd;

	if (freq == 0U) {
		freq = 1U;
	}

	if (_mode == UP_DOWN_COUNT) {
		prd = _tbclk / (2U * freq);
	} else {
		prd = (_tbclk / freq);
		if (prd > 0U) {
			prd -= 1U;
		}
	}

	if (prd == 0U) {
		prd = 1U;
	}
	if (prd > 0xFFFFU) {
		prd = 0xFFFFU;
	}

	return (Uint16) prd;
}

Uint16 EPwmChannel::calculateCMP(Uint16 prd, float duty) {
	if (duty < 0.0f) {
		duty = 0.0f;
	} else if (duty > 1.0f) {
		duty = 1.0f;
	}

	if (_mode == UP_DOWN_COUNT) {
		return (Uint16) ((float) prd * (1.0f - duty));
	}
	return (Uint16) ((float) prd * duty);
}
