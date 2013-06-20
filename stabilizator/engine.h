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





//inicjalizuje silnik
Engine::Engine(int pin) : PIN(pin), MAX_SPEED(50)
{
	this->MOTOR.attach(PIN, 150, 2500); //ustawiam PIN, przez ktory idzie sygnal, a takze dlugosc minimalnego i maksymalnego sygnalu w mikrosekundach
	this->MOTOR.write(0);
	this->is_active = false;
	this->speed = 0;
}

//pobiera numer PINu
const int Engine::getPIN()
{
	return this->PIN;
}

//nadaje okreslona predkosc docelowa temu silnikowi (w rzeczywistosci value = 0..90, lecz dla testow ograniczylismy przez MAX_SPEED do 0..50)
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

//uruchamia ten silnik z nadana wczesniej predkoscia
void Engine::start()
{
	this->is_active = true;
	this->MOTOR.write(this->speed);
}

//zatrzymuje silnik
void Engine::stop()
{
	this->is_active = false;
	this->MOTOR.write(0);
}

#endif