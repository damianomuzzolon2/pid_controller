/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
 public:
  /**
   * TODO: Create the PID class
   **/

  /*
   * Errors
   */
  double cte;
  double diff_cte;
  double int_cte;

  /*
   * Coefficients
   */
  double kp;
  double ki;
  double kd;

  /*
   * Output limits
   */
  double output_lim_min;
  double output_lim_max;

  /*
   * Delta time
   */
  double delta_t;

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
  void Init(double Kpi, double Kii, double Kdi, double output_lim_maxi,
            double output_lim_mini);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double new_cte);

  /*
   * Calculate the total PID error.
   */
  double TotalError();

  /*
   * Update the delta time.
   */
  double UpdateDeltaTime(double new_delta_time);
};

#endif  // PID_CONTROLLER_H
