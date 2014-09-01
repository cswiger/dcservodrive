dcservodrive
============

Arduino sketch to turn step / direction signals into DC servo motor position control - 'stepper simulator' 

Requires libraries:
FlexiTimer2.h   http://playground.arduino.cc/Main/FlexiTimer2<br/>
PID_v1.h        http://playground.arduino.cc/Code/PIDLibrary<br/>
AFMotor.h       my own custom version hacked to work with ardumoto from sparkfun:  https://www.sparkfun.com/products/9815<br/>
Encoder.h       http://playground.arduino.cc/Main/RotaryEncoders</br>

For only one motor, the basic arduino Uno will work, but for more than one motor something like a mega2560 is needed so the interrupts and motor pwm don't conflict. 

