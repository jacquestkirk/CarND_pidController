#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->sumCte = 0;
	this->oldCte = 0;
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

