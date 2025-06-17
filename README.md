# IC102P-FDP
Developed   an autonomous robot that can smoothly follow a black line path using a sophisticated PID (Proportional-Integral-Derivative) control algorithm.

 Implementation of PID controller:
We implemented a line following robot using IR sensors and  utilized a PID (Proportional-Integral-Derivative) controller to optimize the error, which we defined as the distance from the center of the line.
 We employed four IR sensors, positioning the middle two to detect whether the robot was directly on the line. This configuration enhanced the stability of the system, ensuring more accurate line following despite the limitations of the discrete sensor setup.
 
In a line follower robot using IR sensors, the error calculation is critical for determining how far the robot is from the desired line path. With four IR sensors, let's denote them as S1, S2, S3, and S4 from left to right. The middle two sensors, S2 and S3, are used to detect if the robot is on the line.

Sensor Readings: Each sensor outputs a value indicating whether it detects the line (1) or not (0). For simplicity, assume the line is a single value like a black line on a white surface.

Error Calculation: The error can be calculated based on the readings of S2 and S3. If both sensors detect the line, the error is zero (robot is centered). If only S2 or S3 detects the line, the error is positive or negative, respectively. If neither detects the line, the robot may be significantly off the path, and additional logic is needed to determine the direction.

In our robot, the PID controller takes the calculated error as input and adjusts the motor speeds accordingly to correct the path. The motor speed adjustments help steer the robot back towards the line, ensuring accurate and stable line following.
This PID control mechanism allows the robot to smoothly follow the line, correcting its path based on real-time sensor inputs and the calculated error. Adjusting Kp, Ki, and Kd values is crucial for achieving optimal performance.

## Here is a demonstration of the mini robo

[![Curved Lines](https://img.youtube.com/vi/1QRYZWzfOks/0.jpg)](https://www.youtube.com/watch?v=1QRYZWzfOks)
[![Crossing Lines](https://img.youtube.com/vi/77Q0gCPeaZU/0.jpg)](https://www.youtube.com/watch?v=77Q0gCPeaZU)



