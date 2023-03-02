# Position control of a vehicle on rotating ramp using STM32

Goal of the project was to make an object (in this case a small LEGO car) stop at given place on the ramp. The difficulty comes from the fact that it's a non-linear system.

<p allign="center">
  <img src="https://user-images.githubusercontent.com/46852756/222527190-8ba5d930-e3f3-4786-8603-3553c9b4a8ce.gif">
</p>

NUCLEO-F746ZG board was connected with an ultrasonic distance sensor HC-SR04 and servo TowerPro MG-995 - standard. Additional power supply was needed because the servo needed more power than the board could deliver. Also there was a 7-segment display to make it easier for user to read current position of the object.
<p align="center">
  <img src="https://user-images.githubusercontent.com/46852756/222533748-8fb6267a-45a4-4c68-bac4-7e24bc5ff598.png" height="600">
</p>

Data from the microcontroller was sent to PC using USART and user could set the distance (using for example terminal) for the vehicle to stop at or use [Matlab script](https://github.com/4MC4/Position-control-of-a-vehicle-on-a-ramp-using-STM32/blob/main/Real_time_measurement_and_plotting.m) to plot position during last 20 seconds.

PID controller was implemented in the STM32 to make everything possible. Parameters were calculated by using autotune option in Simulink after modeling the object (model was created using the step response for the object).
