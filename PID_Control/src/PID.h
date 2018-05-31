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
  double total_error;
  double min_error;
  double max_error;


  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Auxilary
  */
  bool dbgPrint;
  double cte;
  double cte_t_1;
  int loop_counter;
  std::vector<double> dp;
  int validation_steps;
  int dwell_steps;
  bool twiddle_increase;
  bool twiddle_decrease;
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
  double TotalError(bool print, bool twiddle);
};

#endif /* PID_H */
