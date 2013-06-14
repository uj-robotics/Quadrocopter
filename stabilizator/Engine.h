class Engine
{
private:
	const int PIN;
	Servo MOTOR;
	bool isActive;
	int speed;
        const int MAX_SPEED;

public:
	Engine(int);
	const int GetPIN();
	void SetSpeed(int);
	void Start();
	void Stop();
};

//initializes motor
Engine::Engine(int pin) : PIN(pin), MAX_SPEED(55)
{
	this->MOTOR.attach(PIN, 150, 2500);
	this->MOTOR.write(0);
	this->isActive = false;
	this->speed = 0;
}

//gets this engine's signal PIN
const int Engine::GetPIN()
{
	return this->PIN;
}

//sets desired speed to this engine
void Engine::SetSpeed(int value)
{ 
	if(value < 0)
		value = 0;
	else if(value > MAX_SPEED)
		value = MAX_SPEED;

	this->speed = value;

	if(isActive)
		this->MOTOR.write(this->speed);

       // if(value == 0)
       //    this->isActive = false;
}

//launches this engine
void Engine::Start()
{
	this->isActive = true;
	this->MOTOR.write(this->speed);
}

//stops this engine
void Engine::Stop()
{
	this->isActive = false;
	this->MOTOR.write(0);
}
