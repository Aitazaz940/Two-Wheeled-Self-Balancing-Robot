
# **Two-Wheeled Self-Balancing Robot**

This repository contains the code files and libraries required to implement a Two-Wheeled Self-Balancing Robot.

## Main Working   
ESP32 is the microcontroller.   
Three PID controllers are designed. One for MPU-6050 and two indivdual PID's to control the motor speed.   
The output of main PID (MPU-6050) is given to the motor PID's as the target speed, to keep the robot balanced.   
Robot is interfaced with Dabble app to control the movement of the robot using ESP32 builtin bluetooth.

# **Components Required:**  
* ESP32 Microcontroller  
* MPU-6050 Gyroscope and Accelerometr  
* L298n Motor Driver  
* Two Encoder Motors  
* 12V Battery Power  
