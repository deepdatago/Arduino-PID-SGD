# Arduino-PID-SGD
version 0.0.1 by Mingjun Zhu (deepdatago@gmail.com)

An Arduino PID library which is based on Stochastic Gradient Descent - a Reinforcement Learning experiment.

This work is based on [Brett Beauregard's project](https://github.com/br3ttb/Arduino-PID-Library/)

The key differences are:
* No need (even it still can) to pass in P/I/D value.  The constructor will set them to 1.0, then it will invoke ::CalcSGD() to learn the P/I/D values.  User can still pass in inital P/I/D values by calling ::SetTunings().
* Reset total output sum to zero when the input has passed the setpoint, or when it's within the "max loss" value from setpoint

### Default Values
* sample time: "100" millisecond
* P/I/D set to "1.0"

### TODO - when mP_On_E is 0
for mP_On_E == 0, or P_ON_M is on, I haven't tested this case so I didn't do learning.  This case should be reviewed in future.

### Examples:
Initialize PID_SGD
```
	double Input = 0.0; // variable we want to control
	double Output = 0.0; // variable that will be adjusted
	double Setpoint = 0.0; // value we want input to maintain
	int P_ON_E=1;
	double MaxLoss = 0.1;
	double LearningRate=0.001;
	PID_SGD myPID(&Input, &Output, &SetPoint, P_ON_E, DIRECT, MaxLoss, LearningRate);
```

Set initial PID value.  This is useful to learn from previous experience
```
	double kp = 2.5; // (P)roportional
	double ki = 5.1; // (I)ntegral
	double kd = 1.3; // (D)erivative
	myPID.SetTunings(kp, ki, kd);
```

Set output limit
```
	double MIN = -50; // Default is 0
	double MAX = 100; // Default is 255
	myPID.SetOutputLimits(MIN, MAX);
```

Set learning rate, max loss, start/stop learning
```
	myPID.SetLearningRate(0.0001); // Reduce learning rate is useful when it sways around the setpoint
    
	myPID.SetMaxLoss(0.01); // Learning will stop when the error is smaller than max loss value
    
	myPID.SetLearningFlag(true); // stop learning
    
	myPID.SetLearningFlag(false); // start learning, default behavior
```

Compute output in loop()
```
	myPID.Compute();
```
