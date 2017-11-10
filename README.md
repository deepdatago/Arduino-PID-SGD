# Arduino-PID-SGD
version 0.0.1 by Mingjun Zhu

An Arduino PID library which is based on Stochastic Gradient Descent - a Reinforcement Learning experiment.

This work is based on Brett Beauregard's project: https://github.com/br3ttb/Arduino-PID-Library/

The key differences are:
* No need (even it still can) to pass in P/I/D value.  The constructor will set them to 1.0, then it will invoke ::CalcSGD() to learn the P/I/D values
* Reset total output sum to zero when the input has passed the setpoint, or when it's within the "max loss" value from setpoint

### TODO - mP_On_E==0
for mP_On_E == 0, or P_ON_M is on, I haven't tested this case so I didn't do learning.  This case should be reviewed in future.


