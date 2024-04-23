# Maze Solving Robot

## Team Members
- Mark VanDyke
- Gannon Brady
- Thomas Green
- Alex Rayburn

## Overview
Our robot consists of the STM microcontroller, an ultrasonic sensor, two motor driver boards, and a battery pack mounted onto a two-wheeled robot chassis using 3D printed parts.  
It is able to solve an arbitrary maze as long as the maze fits on a grid and there are no loops. It does this very simply by following the left-hand wall until it reaches the exit.  
As it solves the maze the first time, it records the actions it took to calculate the optimal path. Future runs of the maze use this record to solve it without making any wrong turns.

## How To Use The Robot
1. Set the robot one grid cell away from the maze entrance.
2. Press the user button. The robot will then solve the maze and record its path.
3. Press the user button again when the robot exits the maze to tell it that the solve is done.
4. Move the robot back to the same starting position.
5. Press the user button and the robot will solve the maze again without making any wrong turns.

## Relevant Embedded Systems Concepts Used
- GPIO: Five pins were used to control the motor directions and read the button input. Other pins were set into alternate function mode for other purposes.
- Timers: Four timers were used; one in PWM mode to trigger periodic sensor readings, two in PWM mode to control motor speeds, one in input capture mode to read the sensor result.
- Interrupts: The input capture timer triggers an interrupt used to read the value captured by the timer.
- USART: A USART connection was very useful to display debugging messages during development.
- Motor driver: This project reused a lot of the code from Lab 7 Motor Driver.

## Code Structure
The maze solving algorithm and most of the peripheral setup is in main.c.  
motor.c contains the motor driver setup as well as helper functions to execute the basic commands forward, backward, left, right, and stop.

## Known Issues
Given our limited time to complete the project, we had to ignore several issues.  
The chassis we used was cheap and the wheels are uneven. The motors did not come with built in encoders. The movement is not exact due to it using hardcoded timing delays.  
These movement issues could be fixed with more sensors and PID control.
**We would have fixed these problems if we had another month to work on it.**


