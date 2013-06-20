#ifndef ENGINESMANAGER_H
#define ENGINESMANAGER_H

#include "engine.h"

class EnginesManager
{
private:
	EnginesManager() : FL(Engine(23)), FR(Engine(25)), BL(Engine(22)), BR(Engine(24)) {}

public:
	Engine FL, FR, BL, BR;	//silniki FL = front-left, BR = back-right, itd

	//zwraca obiekt klasy singleton
	static EnginesManager& getEnginesManager()
	{
		static EnginesManager em;
		return em;
	}
};

#endif
