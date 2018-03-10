#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{
	ResetPid(Kp, Ki, Kd);
	this->bestError = 9999999;
	this->bestPid[0] = Kp;
	this->bestPid[1] = Ki;
	this->bestPid[2] = Kd;
}

void PID::ResetPid(double Kp, double Ki, double Kd) 
{
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->sumCte = 0;
	this->oldCte = 0;
	this->mse = 0;
	this->count = 0;
	this->firstMeasurement = true;
}

void PID::UpdateError(double cte) {
	if (this->firstMeasurement)
	{
		this->oldCte = cte;
		this->firstMeasurement = false;
	}
	this->sumCte = this->sumCte + cte;
	double diffCte = cte - this->oldCte;
	this->oldCte = cte;

	this->p_error = -this->Kp * cte;
	this->d_error = -this->Kd * diffCte;
	this->i_error = -this->Ki * sumCte;
}

double PID::TotalError() {
	return this->p_error + this->i_error + this->d_error;
}

void PID::UpdateMse(double cte)
{
	//add this cte to the mse term
	this->mse += cte * cte;
	this->count++;
}

void PID::IncrementPidIndex()
{
	this->pidIndex += 1;
	if (this->pidIndex >= 3)
	{
		this->pidIndex = 0;
	}
}
void PID::Twiddle()
{
	double norm_mse = this->mse / this->count;

	//stop optimizing when this point is reached
	if (dPid[0] + dPid[1] + dPid[2] <= .0001)
	{
		return;
	}

	std::cout << " P: " << this->Kp << " I: " << this->Ki << " D: " << this->Kd << " MSE: " << norm_mse << std::endl;

	//pack into an array
	double nextPid[3];
	nextPid[0] = this->Kp;
	nextPid[1] = this->Ki;
	nextPid[2] = this->Kd;


	//Twiddle State Machine
	switch (this->currentSate)
	{
	case(twiddleState_Initial):
		if (norm_mse < this->bestError) //If this option is better
		{
			this->bestError = norm_mse;
		}
		nextPid[this->pidIndex] += this->dPid[this->pidIndex];
		this->nextState = twiddleState_Decrement;
		break;

	case(twiddleState_Decrement):
		if (norm_mse < this->bestError) //If this option is better
		{
			this->bestError = norm_mse;
			this->dPid[this->pidIndex] *= 1.1;


			//move on to the next parameter
			this->IncrementPidIndex();

			//Increment next parameter
			nextPid[this->pidIndex] += this->dPid[this->pidIndex];
			this->nextState = twiddleState_Decrement;
			break;
		}
		else
		{
			nextPid[this->pidIndex] -= 2 * this->dPid[this->pidIndex]; //decrement by 2 because the last time we incremented by 1, net effect is decrement by 1
			this->nextState = this->twiddleState_ReduceStep;
		}
		break;
	case(twiddleState_ReduceStep):
		if (norm_mse < this->bestError) //If this option is better
		{
			this->bestError = norm_mse;
			this->dPid[this->pidIndex] *= 1.1;

		}
		else
		{
			//revert to the intial settings
			nextPid[this->pidIndex] += this->dPid[this->pidIndex];

			//decrement the step
			this->dPid[this->pidIndex] *= 0.9;

		}

		//move on to the next parameter
		this->IncrementPidIndex();

		//Increment next parameter
		nextPid[this->pidIndex] += this->dPid[this->pidIndex];
		this->nextState = twiddleState_Decrement;
		break;
	}
	this->currentSate = this->nextState;
	this->ResetPid(nextPid[0], nextPid[1], nextPid[2]);
	return;

	
}

