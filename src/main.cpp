#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "twiddle.h"

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

int main(int argc, char *argv[])
{
  uWS::Hub h;
  PID pid;
  uint32_t twiddle_trial_samples = 3000;
  uint32_t twiddle_on = false;
  double coeffs[3] = {0};
  float target_speed = 0;

  // parse PID parameters from command line
  if (argc > 4) {
    for (int i = 0; i < 3; i++) coeffs[i] = std::stod(argv[i+1]);
    pid.Init(coeffs[0], coeffs[1], coeffs[2]);
    target_speed = std::stof(argv[4]);
    printf("PID Init values, Kp %f Ki %f Kd %f\r\n", pid.Kp, pid.Ki, pid.Kd);
    printf("target_speed %f\r\n", target_speed);
  } else { // initialise controller with sensible default
    pid.Init(0.08, 0.0001, 4);
    target_speed = 100;
  }

  // parse twiddle algorithm parameters from command line
  if (argc >= 11) {
    double d_coeffs[3] = {std::stod(argv[5]),
                          std::stod(argv[6]),
                          std::stod(argv[7])};
    double twiddle_descent_ratio = std::stod(argv[8]);
    double tolerance = std::stod(argv[9]);
    twiddle_trial_samples = std::atoi(argv[10]);
    twiddle_init(coeffs, d_coeffs, twiddle_descent_ratio, tolerance);
    twiddle_on = true;
    printf("d_coeffs %f %f %f\r\n", d_coeffs[0], d_coeffs[1], d_coeffs[2]);
    printf("twiddle_descent_ratio %f\r\n", twiddle_descent_ratio);
    printf("tolerance %f\r\n", tolerance);
    printf("twiddle_trial_samples %u\r\n", twiddle_trial_samples);
  }

  h.onMessage([&pid, twiddle_on, twiddle_trial_samples, target_speed]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    static double distance = 0;
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
          // std::cout << j[1] << std::endl;
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          // double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          // use PID controller to set steering_value based on the cte
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          if (twiddle_on) { // run twiddle algorithm
            // infer the distance travelled by the car on road by integrating the speed.
            if (fabs(cte) < 4.2) {
              distance += speed;
            }

            // fail early if there is no proportional control or if car is stuck
            if (pid.Kp == 0.0 || (distance > 10 && speed < 0.1)) {
              // extrapolate error
              pid.accumulative_error = pid.accumulative_error/pid.counter * twiddle_trial_samples;
              pid.counter = twiddle_trial_samples;
            }

            // end of trial run, update results to twiddle engine
            if (pid.counter == twiddle_trial_samples) {
              // devise the goodness parameter for twiddle
              double goodness = pid.accumulative_error / distance;
              twiddle_set_goodness(goodness);

              // reset distance before next trail run
              distance = 0;

              // start twiddle state machine
              twiddle_state_machine(ws, pid);
            }
          }

          // trim steering_value to [-1 1]
          if (steer_value < -1) steer_value = -1;
          if (steer_value > 1) steer_value = 1;

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " ";
          // std::cout << " " << pid.p_error << " " << pid.i_error << " " << pid.d_error << std::endl;

          // some simple rules for controlling throttle
          float throttle = 0;
          if (fabs(cte) > 4.2 && !twiddle_on && speed > 10) { // going off road
            printf("break\r\n");
            throttle = -1;
          } else if (cte/fabs(cte)*pid.d_error > 0.05  && !twiddle_on && speed > 10) { // error increasing rapidly
            printf("break slightly\r\n");
            throttle = -0.5;
          } else if (speed < target_speed) { // speed too slow
            throttle = 1;
          }

          // return control signals back to the simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
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
    (void ) h;
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    (void ) h;
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
