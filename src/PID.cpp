#include "PID.h"

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <functional>
#include <numeric>

using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::setw;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, 
               double p_error_, double i_error_, double d_error_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = p_error_;
  i_error = i_error_;
  d_error = d_error_;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}

void PID::Twiddle(){
  vector<double> p = {0.0, 0.0, 0.0}; //{tau_p, tau_d, tau_i}
  vector<double> dp = {1.0, 1.0, 1.0};

  VEHICLE vehicle_temp = vehicle;
  double best_err = Run(vehicle_temp, p);
  double err;

  while(std::accumulate(p.begin(),p.end(),0) > TOL_TWIDDLE) {
    for(int i=0; i<p.size(); i++) {

      p[i] += dp[i];
      vehicle_temp = vehicle;
      err = Run(vehicle_temp, p);

      //check the error of +dp[i] side
      if(err < best_err) {
        best_err = err;
        dp[i] *= 1.1;
      }

      else {
        //check the error of -dp[i] side
        p[i] -= 2*dp[i];
        vehicle_temp = vehicle;
        err = Run(vehicle_temp, p);
        if(err < best_err) {
          best_err = err;
          dp[i] *= 1.1;
        }
        //if neither error of -dp[i] nor +dp[i] side < best_err
        //assign to smaller dp[i]
        else {
          p[i] += dp[i]; //assign back to original p[i]
          dp[i] *= 0.9; //change dp to smaller number
        }
      }
    }
  }

  //output
  Kp = p[0];
  Kd = p[1];
  Ki = p[2];

  //DEBUG
  int width = 15;
  cout << "p = ";
  for(int i=0; i<p.size(); i++) {
    cout << setw(width) << p[i];
  }
  cout << endl;

  cout << "dp = ";
  for(int i=0; i<dp.size(); i++) {
    cout << setw(width) << dp[i];
  }
  cout << endl;

  cout << "best_err = " << best_err << endl;
  
}


/**
 * @return average error of the give initial position & parameter
 * 
 */
double PID::Run(VEHICLE &vehicle_temp, vector<double> &params) {

  int n = 100; //number of cycle before the current vehicle position

  double speed_ms = vehicle_temp.speed / 2.237; //[m/s], the speed of the car 
    //(/2.237) convert [MPH] to [m/s]
  double error = 0;
  double prev_cte = vehicle_temp.y;
  double int_cte = 0;
  double dT = 1; //[s] the time difference between n to n+1

  for(int i=0; i<(2*n); i++) {
    double cte = vehicle_temp.y;
    double diff_cte = cte - prev_cte;
    int_cte += cte;
    prev_cte = cte;
    double steering = - params[0] * cte - params[1] * diff_cte - params[2] * int_cte;
    double distance = speed_ms * dT;
    Move(vehicle_temp, steering, distance);
    if(i >= n) {
      error += cte*cte;
    }
  }  

  return error/n;
}

/**
 * update the x,y location of the vehicle_temp
 * @param steer front wheel steering angle, limited by max_steering_angle
 * @param distance total distance driven, most be non-negative
 */
void PID::Move(VEHICLE &vehicle_temp, double steering, double distance) {
  
  if(steering > MAX_STEERING_ANGLE) {
    steering = MAX_STEERING_ANGLE;
  }
  if(steering < -MAX_STEERING_ANGLE) {
    steering = -MAX_STEERING_ANGLE;
  }
  if(distance < 0.0) {
    distance = 0.0;
  }
  

}