class Timer
{
private:
	volatile unsigned long lastTime;
	volatile unsigned long now;
        
	Timer() : SAMPLING_MS(30.0), PID_MS(50.0) {}

public:
        const double SAMPLING_MS;
        const double PID_MS;
        
	static Timer& getTimer()
	{
		static Timer instance;
		return instance;
	}

	void startTimer(Tc* TC, uint32_t CHANNEL, IRQn_Type IRQ, double MS)
	{
		pmc_set_writeprotect(false); //Wyłączam write protect na rejestrach związanych z zarządzaniem energią
		pmc_enable_periph_clk((uint32_t)IRQ);  //Włączam zegar o określonym IRQ

		uint32_t rc = VARIANT_MCK/128/(1000.0 / MS); 
		//uint8_t clock = bestClock(1000.0 / MS, rc); //Dobieram najlepszy zegar w zależności od częstotliwości

		TC_Configure(TC, CHANNEL, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);

		TC_SetRA(TC, CHANNEL, rc/2);
		TC_SetRC(TC, CHANNEL, rc);
		TC_Start(TC, CHANNEL);

		TC->TC_CHANNEL[CHANNEL].TC_IER=TC_IER_CPCS;
		TC->TC_CHANNEL[CHANNEL].TC_IDR=~TC_IER_CPCS;
		NVIC_ClearPendingIRQ(IRQ);
		NVIC_EnableIRQ(IRQ);  //Włączam przerwania na określonym IRQ
	}
};
