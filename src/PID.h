#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * static variables
  */
  double oldCte;
  double sumCte;

  bool firstMeasurement;

  double mse;
  double count;
  double bestError;
  int pidIndex = 0;
  double bestPid[3];
  double dPid[3] = { 1,.01,1 };



  enum twiddleStates { twiddleState_Increment, twiddleState_Decrement, twiddleState_ReduceStep, twiddleState_Initial};
  twiddleStates currentSate = twiddleState_Initial;
  twiddleStates nextState;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void UpdateMse(double cte);
  void Twiddle();
  void ResetPid(double Kp, double Ki, double Kd);
  void IncrementPidIndex();
};

#endif /* PID_H */
