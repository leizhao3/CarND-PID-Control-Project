#ifndef PID_H
#define PID_H

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

struct VEHICLE {
   double x;
   double y;
   double speed; //[mph] the speed of the car at that moment
   double angle_rad; //[rad] the angle of the orientation of the vehicle
   double length; //[meter] length of the vehicle
};

class PID {
 public:
      /**
      * Constructor
      */
      PID();

      /**
      * Destructor.
      */
      virtual ~PID();

      /**
      * Initialize PID.
      * @param (Kp_, Ki_, Kd_) The initial PID coefficients
      */
      void Init(double Kp_, double Ki_, double Kd_, 
               double p_error_, double i_error_, double d_error_);

      /**
      * Calculate the cte_del
      * @param cte The current cross track error
      */
      void DelError(double cte);

      /**
      * Calculate the total PID error.
      * @param cte The current cross track error
      */
      void TotalError(double cte);

      /**
      * @return the steering needed for car to manuver. 
      * @param cte The current cross track error
      */
      double GetSteering(double cte);

      /**
      * Calculate the optimized parameter(minimized error) for the PID controller
      * @return 
      */
      void Twiddle();


      /**
       * Vechile class
       */ 
      VEHICLE vehicle;


      /**
       * @return average error of the give initial position & parameter
       */
      double Run(VEHICLE &vehicle_temp, std::vector<double> &params);

      /**
       * update the x,y location of the vehicle_temp
       */
      void Move(VEHICLE &vehicle_temp, double steering, double distance);





 private:
      /**
       * PID Errors
       */
      double p_error;
      double i_error;
      double d_error;

      /**
       * PID Coefficients
       */ 
      double Kp;
      double Ki;
      double Kd;

      //Constants
      const double MAX_STEERING_ANGLE = 1;
      const double TOL_STRAIGHT_LINE = 0.001; //[rad]
      const double TOL_TWIDDLE = 0.001; //[unitless] 

      /**
       * CTE Errors 
       */
      double cte_sum;
      double cte_del;
      double cte_prev;
      bool cte_prev_initilized = false;

      

};


#endif  // PID_H