#ifndef TIMER_H
#define TIMER_H

class Timer
{
private:
	volatile unsigned long lastTime;
	volatile unsigned long now;

	Timer() : SAMPLING_MS(30.0), PID_MS(50.0) {}

public:
	const double SAMPLING_MS;
	const double PID_MS;

	//returns singleton object
	static Timer& getTimer()
	{
		static Timer instance;
		return instance;
	}

	void startTimer(Tc*, uint32_t, IRQn_Type, double);
};




/*
* Starts one of the timers
*/
void Timer::startTimer(Tc* TC, uint32_t CHANNEL, IRQn_Type IRQ, double MS)
{
	pmc_set_writeprotect(false);	//disables write protect on power management registers
	pmc_enable_periph_clk((uint32_t)IRQ);	//turns on clock with specified IRQ

	int32_t rc = VARIANT_MCK / 128 / (1000.0 / MS);	//CLOCK4 is our clock, 1/128 is its scale, (1000.0 / MS) is our frequency in Hz

	TC_Configure(TC, CHANNEL, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4); //configure time counter on specified channel, wave and clock4
	TC_SetRA(TC, CHANNEL, rc/2);
	TC_SetRC(TC, CHANNEL, rc);
	TC_Start(TC, CHANNEL);

	TC->TC_CHANNEL[CHANNEL].TC_IER = TC_IER_CPCS;
	TC->TC_CHANNEL[CHANNEL].TC_IDR = ~TC_IER_CPCS;
	NVIC_ClearPendingIRQ(IRQ);
	NVIC_EnableIRQ(IRQ);	//turns on interrupts in interrupts controller with specified IRQ
}

#endif
