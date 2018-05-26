#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

		  // convert ptsx from global cooridinate to vehicle coordinate
		  const int NUMBER_PTS = ptsx.size(); //number of ptsx/ptsy points
		  
		  Eigen::VectorXd ptsx_VecCoor(NUMBER_PTS);
		  Eigen::VectorXd ptsy_VecCoor(NUMBER_PTS);

		  for (int i = 0; i < NUMBER_PTS; ++i)
		  {
			  double dx = ptsx[i] - px;
			  double dy = ptsy[i] - py;

			  ptsx_VecCoor[i] = dx * cos(-psi) - dy * sin(-psi);
			  ptsy_VecCoor[i] = dy * cos(-psi) + dx * sin(-psi);
		  }

		  //current state and coeffs
		  int fit_order = 3; //fit tripnomial polyfit
		  auto coeffs = polyfit(ptsx_VecCoor, ptsy_VecCoor, fit_order);

		  const double cte = coeffs[0];
		  const double epsi = -atan(coeffs[1]);

		  //latency: current state for px, py, psi=0.0;
		  //however, steering and acceleration or decleration acurator
		  //have a latency, so at that time, the current state changes
		  //in this project, the latency is 0.1 which equals to dt
		  //so, all states should be calculated by kinematic model
		  //and then input our MPC
		  //that's the good performance which MPC can handle latency of acuator
		  //note: in vehicle coordinate, vehicle only change x, and y keep constant 0
		  double delta = j[1]["steering_angle"];
		  double a= j[1]["throttle"];
		  
		  const double Lf = 2.67;
		  double dt_main = 0.1;
		  size_t N_main = 10;

		  const double x_next = 0.0 + v * dt_main;
		  const double y_next = 0;
		  const double psi_next = 0.0 + v * (-delta) / Lf * dt_main;
		  const double v_next = v + a * dt_main;
		  const double cte_next = cte + v * sin(epsi)*dt_main;
		  const double epsi_next = epsi + v * (-delta) / Lf * dt_main;

		  const int NUMBER_OF_STATES = 6;// 6 states: x, y, psi, v, cte, epsi
		  Eigen::VectorXd states(NUMBER_OF_STATES);
		  states << x_next, y_next, psi_next, v_next, cte_next, epsi_next;
		  
		  //using MPC
		  mpc.Solve(states, coeffs);

		  double steer_value = mpc.steer;
		  double throttle_value = mpc.throttle;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          // vector<double> mpc_x_vals;
          // vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

		  msgJson["mpc_x"] = mpc.x_mpc;
          msgJson["mpc_y"] = mpc.y_mpc;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

		  for (unsigned int i = 0; i < N_main; ++i)
		  {
			  double x = 5.0 * i; // show each 5 points
			  double y = polyeval(coeffs, x);

			  next_x_vals.push_back(x);
			  next_y_vals.push_back(y);
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
