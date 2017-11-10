/**********************************************************************************************
 * Arduino PID Library with Stochastic Gradient Decent - Version 0.0.1
 * by Mingjun Zhu <deepdatago@gmail.com>
 * based on the work from Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * Brett's work is at https://github.com/br3ttb/Arduino-PID-Library/
 *
 * Major changes are:
 *
 * 1) Add CalcSGD, which is to use Stochastic Gradient Descent method to learn the P/I/D
 *
 * 2) Add ResetTotal method.  This is to reset ouput sum to 0, when error is less than
 *    the predefined max loss, or when error sways pass the setpoint
 *
 * TODO: for (!mP_On_E), or P_ON_M is on, I haven't tested this case so I didn't do learning.
 * This case should be reviewed in future
 *
 * Note: naming convention on variables:
 * 	"m" stands for member
 * 	"b" stands for boolean
 * 	"mb" stands for member boolean
 * 	"i" stands for input
 * 	"ip" stands for input pointer
 * 	"r" stands for reference
 * 	"ir" stands for input reference
 * 	"l" stands for local
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "PID_SGD.h"

PID_SGD::PID_SGD(double* ipInput,
	double* ipOutput,
	double* ipSetpoint,
        int iP_On,
	int iControllerDirection,
	double iMaxLoss,
	double iLearningRate)
{
	mMaxLoss = iMaxLoss;
	mKpLearningRate = iLearningRate;
	mKiLearningRate = iLearningRate;
	mKdLearningRate = iLearningRate;

	// PID are set to 1.0 by default, user can call SetTunings to give different
	// initial values
	mKp = 1.0;
	mKi = 1.0;
	mKd = 1.0;

	mLastDInput = 0.0;
	mTotalError = 0.0;

	mTotalErrorBeforeLastErrorPoint = 0;
	mLastErrorPoint = millis();
	mLastError = 0;
	mLastOutput = 0.0;
	mbStopLearning = false;
#ifdef PID_TRACE
	mDebugStr = "";
#endif
	mpOutput = ipOutput;
	mpInput = ipInput;
	mpSetpoint = ipSetpoint;
	mbInAuto = false;

	//default output limit corresponds to the arduino pwm limits
	SetOutputLimits(0, 255);

	//default Controller Sample Time is 0.1 seconds
	mSampleTimeInMilliSecond = 100;

	SetControllerDirection(iControllerDirection);
	SetTunings(mKp, mKi, mKd, iP_On);

	mLastTime = millis()- mSampleTimeInMilliSecond;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID_SGD::Compute()
{
#ifdef PID_TRACE
	mDebugStr = "";
#endif
	if(!mbInAuto)
		return false;

	unsigned long now = millis();
	unsigned long timeChange = (now - mLastTime);
	if(timeChange>=mSampleTimeInMilliSecond)
	{
		/*Compute all the working error variables*/
		double input = *mpInput;

		#ifdef PID_TRACE
		// LogTrace(" input: ");
		// LogTrace(input);
		#endif

		double error = *mpSetpoint - input;
		#ifdef PID_TRACE
		// LogTrace(" error: ");
		// LogTrace(error);
		#endif

		if (abs(error) < mMaxLoss)
		{
			*mpOutput = mLastOutput;
			ResetTotal(now);
			return true;
		}

		double dInput = (input - mLastInput);
		#ifdef PID_TRACE
		// LogTrace(" dInput: ");
		// LogTrace(dInput);
		#endif

		double lastOutputSum = mOutputSum;
		#ifdef PID_TRACE
		// LogTrace(" lastOutputSum: ");
		// LogTrace(lastOutputSum);
		#endif

		#ifdef PID_TRACE
		LogTrace("\nPrev mTotalError: ");
		LogTrace(mTotalError);
		LogTrace(" current mTotalError: ");
		LogTrace(mTotalError+error);

		#ifdef PID_TRACE
		LogTrace(" current mKi: ");
		LogTrace(mKi);
		#endif

		#endif

		mKi = CalcSGD(mTotalError, mTotalError + error, mKi, mKiLearningRate);
		#ifdef PID_TRACE
		LogTrace(" Calc mKi: ");
		LogTrace(mKi);
		#endif

		mTotalError += error;

		if ((mLastError / error) < 0)
		{
			// just changed to the opposite error, so previous total error doesn't apply
			ResetTotal(now);
		}
		else
		{
			mOutputSum = mKi * mSampleTimeInMilliSecond / 1000 *  mTotalError;
		}
		#ifdef PID_TRACE
		LogTrace(" mOutputSum1: ");
		LogTrace(mOutputSum);
		#endif


		/*Add Proportional on Measurement, if P_ON_M is specified*/
		if(!mP_On_E)
		{
			// TODO: do we need to call CalcSGD on mKp?
			mOutputSum-= mKp * dInput;
		}

		if(mOutputSum > mOutMax)
			mOutputSum= mOutMax;
		else if(mOutputSum < mOutMin)
			mOutputSum= mOutMin;

		/*Add Proportional on Error, if P_ON_E is specified*/
		double output = 0.0;
		if(mP_On_E)
		{
			mLastError = *mpSetpoint - mLastInput;
			mKp = CalcSGD(*mpSetpoint - mLastInput, error, mKp, mKpLearningRate);
			#ifdef PID_TRACE
			LogTrace(" Calc mKp: ");
			LogTrace(mKp);
			#endif
	
			output = mKp * error;
			#ifdef PID_TRACE
			// LogTrace(" output2: ");
			// LogTrace(output);
			#endif
		}
		else
		{
			output = 0;
		}

		/*Compute Rest of PID Output*/
		mKd = CalcSGD(mLastDInput, dInput, mKd, mKdLearningRate);
		#ifdef PID_TRACE
		LogTrace(" Calc mKd: ");
		LogTrace(mKd);
		#endif

		output += mOutputSum - (mKd / mSampleTimeInMilliSecond / 1000) * dInput;
		#ifdef PID_TRACE
		// LogTrace(" output3: ");
		// LogTrace(output);
		#endif

		mLastDInput = dInput;

		if(output > mOutMax)
			output = mOutMax;
		else if(output < mOutMin)
			output = mOutMin;

		*mpOutput = output;

		#ifdef PID_TRACE
		// LogTrace(" output_final: ");
		// LogTrace(output);
		#endif
		mLastOutput = output;

		/*Remember some variables for next time*/
		mLastInput = input;
		mLastTime = now;
		return true;
	}

	return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID_SGD::SetTunings(double iKp, double iKi, double iKd, int iPOn)
{
	mP_On = iPOn;
	mP_On_E = (iPOn == P_ON_E);

	mKp = iKp;
	mKi = iKi;
	mKd = iKd;

	if(mControllerDirection == REVERSE)
	{
		mKp = (0 - mKp);
		mKi = (0 - mKi);
		mKd = (0 - mKd);
	}
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID_SGD::SetTunings(double iKp, double iKi, double iKd){
	SetTunings(iKp, iKi, iKd, mP_On); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID_SGD::SetSampleTime(int iNewSampleTimeInMilliSecond)
{
	if (iNewSampleTimeInMilliSecond <= 0)
		return;

	mSampleTimeInMilliSecond = iNewSampleTimeInMilliSecond;
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SGD::SetOutputLimits(double iMin, double iMax)
{
	if(iMin >= iMax)
		return;

	mOutMin = iMin;
	mOutMax = iMax;

	if(mbInAuto)
	{
		if(*mpOutput > mOutMax)
			*mpOutput = mOutMax;
		else if(*mpOutput < mOutMin)
			*mpOutput = mOutMin;

		if(mOutputSum > mOutMax)
			mOutputSum= mOutMax;
		else if(mOutputSum < mOutMin)
			mOutputSum= mOutMin;
	}
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID_SGD::SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if(newAuto && !mbInAuto)
	{  /*we just went from manual to auto*/
	        PID_SGD::Initialize();
	}
	mbInAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID_SGD::Initialize()
{
	mOutputSum = *mpOutput;
	mLastInput = *mpInput;
	mLastDInput = *mpInput;

	if(mOutputSum > mOutMax)
		mOutputSum = mOutMax;
	else if(mOutputSum < mOutMin)
		mOutputSum = mOutMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SGD::SetControllerDirection(int iDirection)
{
	if(mbInAuto && iDirection != mControllerDirection)
	{
		mKp = (0 - mKp);
		mKi = (0 - mKi);
		mKd = (0 - mKd);
	}
	mControllerDirection = iDirection;
}

int PID_SGD::GetMode()
{
	return  mbInAuto ? AUTOMATIC : MANUAL;
}

int PID_SGD::GetDirection()
{
	return mControllerDirection;
}

double PID_SGD::CalcSGD(double iPrevFeedback, double iNewFeedback, double iTheta, double iLearningRate)
{
	// English refence: page 7 at: https://mycourses.aalto.fi/pluginfile.php/393629/mod_resource/content/1/Lecture8.pdf
	// or, in Chinese: http://blog.csdn.net/lilyth_lilyth/article/details/8973972
	// simplify it as:
	// h(x) = gTheta * x, h(x) is the PID input for next episode
	// where, x is prevFeedback, can be set to anything (like 0.0) at beginning
	// y is newFeedback, which is the actual feedback from sensor (environment)
	// we need to train theta

	if (mbStopLearning)
	{
		return iTheta;
	}

	if (abs(iTheta * iPrevFeedback - iNewFeedback) < mMaxLoss)
	{
		return iTheta;
	}

	double prevTheta = iTheta;

	// This is the enhancement part: supposely 
	// 
	//	lDiff = iLearningRate * (iTheta * iPrevFeedback - iNewFeedback) * iPrevFeedback;
	// 
	// should be used to calculate lDiff, from theory in the URLs
	// However, I noticed that if input feedback is large, say, in magnitude of 100,
	// then "(iTheta * iPrevFeedback - iNewFeedback) * iPrevFeedback"
	// could end up with very large number, and make the lDiff becomes larger and
	// does not converge.  Therefore, I used "iTheta * iLearningRate" to guarantee
	// that lDiff will converge
	double lDiff = iTheta * iLearningRate;

	// lOrigDiff is used to decide if iTheta should be larger or smaller
	double lOrigDiff = (iTheta * iPrevFeedback - iNewFeedback) * iPrevFeedback;
	if (lOrigDiff < 0)
	{
	  	lDiff = -1 * lDiff;
	}
  
	iTheta = iTheta - lDiff;

	return iTheta;
}

double PID_SGD::GetKp()
{
	return  mKp;
}

double PID_SGD::GetKi()
{
	return  mKi;
}

double PID_SGD::GetKd()
{
	return  mKd;
}

double PID_SGD::GetTotalError()
{
	return  mTotalError;
}

void PID_SGD::LogTrace(double iVal)
{
#ifdef PID_TRACE
	mDebugStr += iVal;
#endif
}

void PID_SGD::LogTrace(char* iStr)
{
#ifdef PID_TRACE
	mDebugStr += iStr;
#endif
}

void PID_SGD::ResetTotal(unsigned long iNow)
{
	mOutputSum = 0;
	mTotalError = 0;
	mLastErrorPoint = iNow;
	mTotalErrorBeforeLastErrorPoint = mTotalError;
}

void PID_SGD::SetLearningFlag(bool ibFlag)
{
	mbStopLearning = ibFlag;
}

void PID_SGD::SetLearningRate(double iLearningRate)
{
	mKpLearningRate = iLearningRate;
	mKiLearningRate = iLearningRate;
	mKdLearningRate = iLearningRate;
}

void PID_SGD::SetMaxLoss(double iLoss)
{
	mMaxLoss = iLoss;
}

double PID_SGD::GetLastError()
{
	return  mLastError;
}

