/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi,
               double output_lim_mini) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  cte = 0.0;
  diff_cte = 0.0;
  int_cte = 0.0;
  kp = Kpi;
  ki = Kii;
  kd = Kdi;
  output_lim_min = output_lim_mini;
  output_lim_max = output_lim_maxi;
  delta_t = 0.0;
}

void PID::UpdateError(double new_cte) {
  /**
   * TODO: Update PID errors based on cte.
   **/
  diff_cte = (new_cte - cte) / delta_t;
  cte = new_cte;
  int_cte += new_cte * delta_t;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   * The code should return a value in the interval [output_lim_mini,
   * output_lim_maxi]
   */
  double control = -(cte * kp + diff_cte * kd + int_cte * ki);
  return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  /**
   * TODO: Update the delta time with new value
   */
  delta_t = new_delta_time;
}