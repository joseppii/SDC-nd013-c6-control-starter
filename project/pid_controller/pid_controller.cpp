/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * Initialize PID coefficients (and errors, if needed)
   **/
   kp_ = Kpi;
   ki_ = Kpi;
   kd_ = Kdi;

   p_err_ = 0.0;
   i_err_ = 0.0;
   d_err_ = 0.0;

   lim_max_ = output_lim_maxi;
   lim_min_ = output_lim_mini;

   delta_t_ = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * Update PID errors based on cte.
   * 
   **/
   if (delta_t_ > 0)
      d_err_ = (cte-p_err_)/delta_t_;
   else
      d_err_ = 0.0;

   p_err_ = cte;
   i_err_ += cte*delta_t_; 
}

double PID::TotalError() {
   /**
   * Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */

   double control;
   control = kp_ * p_err_ + kd_ * d_err_ + ki_ * i_err_;
   if (control < lim_min_)
      control = lim_min_;
   else if(contro>lim_max_)
      control = lim_max_;
   
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with new value
   */
  delta_t_ = new_delta_time;
  return delta_t_;
}