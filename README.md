Quadrocopter Project
===========
##### Developed under Jagiellonian Robotics Association quadrocopter project. 


Current phase : *stabilization*

Current version : quadrocopter is able to stabilize itself

TODO : pick more carefully constants and attach sonar for height regulation

Theory
----------------

We are using simple PID regulator on each axis, along with complementary filter. For futher information on
the filter see [http://web.mit.edu/scolton/www/filter.pdf](http://web.mit.edu/scolton/www/filter.pdf)


Running
-------------------
It is an arduino project. Currently there is no height measurements so constant force is applied (`u_base` variable).


We have also implemented simple 3d visualisation. To run it simply type `python main.py` , but remember to change COM
setting in the code. It requires 3d packages, easiest way would be to install `pythonxy` package bundle for python.


Notes on implementation and callibration
---------------------
It uses 2 separate timers. One is sampling measurements from IMU, and the second one is adjusting position using PID. 

All measurements are accumulated in `ReferenceFrame` class, which can be fetched using `rf.getAngles()` for instance.
 
PID constants are set in `setup()` using function `set_tunings(KP,KI,KD)` .

Measurements from gyro and acc are run through low-pass filter (around 40hz). It can be changed setting different initialization
of gyro (because low pass if builtin), and for acc by setting appropriate constant in `ReferenceFrame` class

IMU is callibrated and hardcoded, however it is easy to callibrate it. Both gyro and acc have to be callibrated for
offset and gain. Compass is very accurate without any callibration. Callibrated constants should be hardcoded in `SensorsManager` class.
It is important that gyro is sensitive to temperature, so it should be kept cool

Also IMU is very sensitive to vibrations, even with strong low pass filters, so it should be attached studirly
