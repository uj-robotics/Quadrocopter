#ifndef ENGINE_H
#define ENGINE_H

class Engine
{
private:
	const int PIN;
	const int MAX_SPEED;
	Servo MOTOR;
	bool is_active;
	int speed;

public:
	Engine(int);
	const int getPIN();
	void setSpeed(int);
	void start();
	void stop();
};





//inits engine
Engine::Engine(int pin) : PIN(pin), MAX_SPEED(50)
{
	this->MOTOR.attach(PIN, 150, 2500); //sets PIN, as well as minimal and maximal signal lengths in [ms]
	this->MOTOR.write(0);
	this->is_active = false;
	this->speed = 0;
}

//gets PIN of this engine
const int Engine::getPIN()
{
	return this->PIN;
}

//set specified speed to engine (usually value = 0..90 but for testing purpose we reduced range by MAX_SPEED to 0..50)
void Engine::setSpeed(int value)
{ 
	if(value < 0)
		value = 0;
	else if(value > MAX_SPEED)
		value = MAX_SPEED;

	this->speed = value;

	if(is_active)
		this->MOTOR.write(this->speed);
}

//starts this engine with the speed declared before
void Engine::start()
{
	this->is_active = true;
	this->MOTOR.write(this->speed);
}

//stops the engine
void Engine::stop()
{
	this->is_active = false;
	this->MOTOR.write(0);
}

#endif