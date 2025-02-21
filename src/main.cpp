#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::setw;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

/**
 * A function that returns a value between 0 and 1 for x in the range [0, infinity] and 
 *                                        -1 to 1 for x in the range [-infinity, infinity].
 */
double logistic(double x) {
  return 2.0 / (1 + exp(-x)) - 1.0;
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(0,0,0, 1,1,1);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {

    const double CTE_MAX = 0.5;

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());//[meter]
          double speed = std::stod(j[1]["speed"].get<string>());//[mph]
          double angle = std::stod(j[1]["steering_angle"].get<string>()); //[deg]
          pid.vehicle = {0.0, cte, speed, deg2rad(angle), 5.0};
          /*//DEBUG
          int width = 15;
          cout << "===============PID Controller===============" << endl;
          cout << setw(width)<<"cte[m]" << setw(width)<<"speed[mph]" << setw(width)<<"str_agl[deg]" << endl;
          cout << setw(width)<<cte << setw(width)<<speed << setw(width)<<angle << endl;*/

          double steer_value_rad = pid.GetSteering(cte);
          double steer_value = steer_value_rad / 0.873;
            //steer_value is [-1, 1] == [-25, +25] deg == [-0.873, +0.873] rad
          
          /*// DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Steering Value[deg]: " << steer_value*25
                    << std::endl;*/

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = std::max(0.3*logistic(CTE_MAX/fabs(cte)),0.1); //reward low cte for higher throttle
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}