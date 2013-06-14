#include "Engine.h"

class EnginesManager
{
public:
	Engine FL, FR, BL, BR;

	EnginesManager();
	EnginesManager(int, int, int, int);
};

//initializes default engines' PINs
EnginesManager::EnginesManager() : FL(Engine(23)), FR(Engine(25)), BL(Engine(22)), BR(Engine(24)) {}

//front-left engine PIN, front-right engine PIN, back-left engine PIN, back-right engine PIN
EnginesManager::EnginesManager(int fl, int fr, int bl, int br) : FL(Engine(fl)), FR(Engine(fr)), BL(Engine(bl)), BR(Engine(br)) {}
