# Canoptima

Final product codes

MCU
The embeded codes for microcontroller which receives messages from UART and sends required signals to the step motors to realize the motion.

FreeTiltSolver_040623
The Matlab code that calculates optimal solution of given light source position in terms of radius, elevation and azimuth given below where The CenterZ variable give the height of the canopy. The Intersection Over Union (IOU) value of the optimal solution is given as output with the 2D view of the initial and subsequent shadow and the 3D view of the canopy with the shadows.

canoptima.py
The python code that takes input from the cameras and calculates the required motion. After the movement is specified then the movement message is generated and sent through UART interface.
