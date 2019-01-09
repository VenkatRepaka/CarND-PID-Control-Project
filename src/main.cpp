#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <limits>

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  // pid.Init(0.2, 0.004, 3.0);
  bool enable_twiddle = false;
  int run = 0;
  double sum_dp, tolerance = 0.001;
  int steps_before_twiddle = 1500, p_iterator = 0;
  double sum_cte = 0.0;
  double p[3] = {0.132, 0.000649039, 4.095}; // Twiddled values
  //double p[3] = {0.12, 0.00065, 4.5};
  // double p[3] = {0.12, 0.0007, 4.0}; 
  // High speed tests
  // double p[3] = {0.105, 0.00065, 2.5}; 
  // double p[3] = {0.105, 0.00065, 2.5};

  // double p[3] = {0.105, 0.00065, 3.5}; // For high speed 2.5
  // double p[3] = {0.12, 0.00065, 4.5};
  // double p[3] = {0.15, 0.00075, 8.0}; // Another best
  // double p[3] = {0.11, 0.0009, 8.0}; // By far best
  // double p[3] = {0.10, 0.0008, 4.0}; // Best in the morning
  // double p[3] = {0.10, 0.0015, 4.0}; // Best till now. Little oscillation but perfect turns
  // double p[3] = {0.10, 0.0002, 3.2}; // Seemed to be best
  // double p[3] = {0.09, 0.00015, 3.0}; // Seems to be decent values only. Have to check with speed. Working on this
  // double p[3] = {0.099, 0.0002, 3.2}; // Same as below at steep turns bt more oscillation at smaller curves
  // double p[3] = {0.099, 0.0002, 3.0}; // Same as below
  // double p[3] = {0.099, 0.0002, 2.8}; // Steep turns were better
  // double p[3] = {0.099, 0.0002, 2.5}; // Good one as below one
  // double p[3] = {0.099, 0.00015, 2.5}; // Felt this took best turn
  // double p[3] = {0.09, 0.00015, 1.8}; - {0.09, 0.00015, 2.5}// Did good at turnings but more oscillation
  // double p[3] = {0.045, 0.00015, 2.5}; {0.045, 0.00015, 2.0} // Out of road at 2nd turning after bridge
  // double p[3] = {0.055, 0.00015, 2.0}; // Not so good at 2nd turning after bridge
  // double p[3] = {0.07, 0.00011, 1.8}; // First reading which crossed 2 laps
  /*
  From 0.07, 0.00011, 1.8 t0 0.07, 0.0002, 1.8 It always passed lap but with increase in oscillation
  */
  // double p[3] = {0.09, 0.00005, 1.3};
  // double p[3] = {0.12, 0.0002, 1.71};
  // double p[3] = {0.071, 0.0001, 1.51};
  // double p[3] = {0.05, 0.0001, 1.5};
  // double p[3] = {0.12, 0.0002, 1.71};
  double dp[3] = {p[0] * 0.1, p[1] * 0.1, p[2] * 0.1};
  // double dp[3] = {0.01, 0.0001, 0.1};
  double best_p[3] = {0.0, 0.0, 0.0};
  double error = 0.0, best_error = std::numeric_limits<double>::max();

  pid.Init(p[0], p[1], p[2]);
  pid.enable_twiddle = enable_twiddle;
  pid.max_steps = steps_before_twiddle;

  PID pid_throttle;
  pid_throttle.Init(0.35, 0.0001, 0.15);

  h.onMessage([&pid, &pid_throttle, &sum_dp, &tolerance, &sum_cte, &p, &dp, &best_p, &error, &best_error, &run, &p_iterator](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>()); // No use of this value
          double steer_value;
          double throttle_value = 0.3;
          double base_throttle = 0.5;
          double max_throttle = 0.75;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          if(pid.enable_twiddle) {
            sum_cte += cte * cte;
            if(pid.steps == 0) {
              // TODO reset PID parameters
              pid.Init(p[0], p[1], p[2]);
            }
            pid.steps += 1;
            pid.UpdateError(cte);
            steer_value = pid.TotalError();

            /* pid_throttle.UpdateError(fabs(cte));
            throttle_value = max_throttle + pid_throttle.TotalError(); */

            if(pid.steps > pid.max_steps) {
              if(run == 0) {
                std::cout << "Run 0 p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
                cout << "Before increment " << p[p_iterator] << endl;
                p[p_iterator] += dp[p_iterator];
                cout << "Incremented " << p_iterator << endl;
                cout << "After increment " << p[p_iterator] << endl;
                std::cout << "Updated in run 0 p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
                run = 1;
                // pid.Init(p[0], p[1], p[2]);
              }
              else {
                error = sum_cte/pid.steps;
                std::cout << "Error: " << error << std::endl;
                std::cout << "Best error: " << best_error << std::endl;
                if(error < best_error && run == 1) {
                  best_error = error;
                  best_p[0] = p[0];
                  best_p[1] = p[1];
                  best_p[2] = p[2];
                  dp[p_iterator] *= 1.1;
                  cout << "dp of " << p_iterator << " is multiplied by 1.1" << endl;
                  cout << "dp values " << dp[0] << " " << dp[1] << " " << dp[2] << endl;
                  p_iterator = (p_iterator + 1) % 3;
                  cout << "next p_iterator is "<< p_iterator << endl;
                  run = 0;
                  std::cout << "Run 1 best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << std::endl;
                }
                else {
                  if(run == 1) {
                    cout << "Before decrement " << p[p_iterator] << endl;
                    p[p_iterator] -= 2 * dp[p_iterator];
                    cout << "Decremented " << p_iterator << endl;
                    cout << "After decrement " << p[p_iterator] << endl;
                    std::cout << "Updated in run 1 p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
                    run = 2;
                  }
                  else {
                    if(error < best_error) {
                      best_error = error;
                      best_p[0] = p[0];
                      best_p[1] = p[1];
                      best_p[2] = p[2];
                      dp[p_iterator] *= 1.1;
                      cout << "dp of " << p_iterator << " is multiplied by 1.1" << endl;
                      cout << "dp values " << dp[0] << " " << dp[1] << " " << dp[2] << endl;
                      std::cout << "Run 2 best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << std::endl;
                    }
                    else {
                      cout << "Before increment " << p[p_iterator] << endl;
                      p[p_iterator] += dp[p_iterator];
                      cout << "Incremented " << p_iterator << endl;
                      cout << "After increment " << p[p_iterator] << endl;
                      dp[p_iterator] *= 0.9;
                      cout << "dp of " << p_iterator << " is multiplied by 0.9" << endl;
                      cout << "dp values " << dp[0] << " " << dp[1] << " " << dp[2] << endl;
                      std::cout << "Updated in run 2 p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
                    }
                    run = 0;
                    p_iterator = (p_iterator + 1) % 3;
                    cout << "next p_iterator is "<< p_iterator << endl;
                  }
                }
              }
              sum_cte = 0.0;
              pid.steps = 0;
              double sum_dp = dp[0] + dp[1] + dp[2];
              std::cout << "sum_dp: " << sum_dp << std::endl;
              if(sum_dp < tolerance) {
                std::cout << "Final best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << std::endl;
                ws.close();
              }
              else {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            }
          }
          else {
            pid.UpdateError(cte);
            steer_value = pid.TotalError();

            // Based on assumption that speed and steering radius are inversely proportional.
            // This suits in the simulator
            double old_steer_val = steer_value;
            double throttle = (1 - steer_value * steer_value * speed) * 0.25 + base_throttle;
            throttle_value = throttle;
            if(throttle < -0.3) {
              steer_value += 0.5 * steer_value;
            }

            // steer_value = angle;
            // cout << "Original steer_value: " << old_steer_val << " steer_value: " << steer_value << " Input Angle: " << angle << " Speed: " << speed << " steer_value * speed: " << (speed * fabs(steer_value)) << " Expected throttle: " << throttle << endl;
            /* double throttle_ratio = ((speed * fabs(steer_value))/5);
            if(throttle_ratio > 1.0) {
              throttle_value = 1 - throttle_ratio;
            } */

            // Throttle updation using PID controller
            /* pid_throttle.UpdateError(fabs(cte));
            throttle_value = max_throttle + pid_throttle.TotalError();
            cout << "cte: " << cte << " throttle_value: " << throttle_value << endl; */
          }

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
