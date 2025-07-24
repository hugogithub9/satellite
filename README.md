This project is designed to process data from the GY-BNO055 sensor, which provides quaternion orientation data. 
The goal is to simulate a control system for a satellite's orientation using the DEVS (Discrete Event System Specification) formalism.

Input : 
The system takes as input quaternion data from the GY-BNO055 sensor, representing the orientation of the satellite in 3D space.
For now, the sensor connection is not working correctly, the board cannot read all data from the GY-BNO055.
To continue development and testing, a temporary Data atomic model was created to simulate quaternion inputs using random values.

DEVS formalism : 
The DEVS model consists of two atomic components:

Calculator
Converts the input quaternion into Euler angles (roll, pitch, yaw).
Sends these angles as output to the second atomic block.
Successfully tested with simulated quaternion data to verify correct orientation computation in "examples".

Analyzer (not implemented yet)
Compares the Euler angles to a given tolerance threshold provided as input.
Depending on whether the orientation exceeds the allowed thresholds on X, Y, or Z axes, it generates a control signal to correct the satellite's position.

Although not yet implemented, the final system will control 3 motors, each responsible for adjusting the satellite's orientation along one axis (X, Y, Z).
Each motor requires 2 values: Direction of rotation and Intensity of the action. 
This results in a total of 6 output values from the system (3 directions + 3 intensities).
