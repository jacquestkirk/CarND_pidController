#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#define MAX_STEPS  4000

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int loop_count;
int stuck_count;
bool firstRun = true;
bool useTwiddle = false;

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
  pid.Init(.306, .00213, 5.508);


  loop_count = 0;
  stuck_count = 0;


#ifdef UWS_VCPKG
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
#else
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
#endif

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
	  if (firstRun)
	  {
		  firstRun = false;
		  std::string msg = "42[\"reset\",{}]"; // thanks to https://discussions.udacity.com/t/how-to-implement-twiddle-optimisation/279749/13
#ifdef UWS_VCPKG
										 // code fixed for latest uWebSockets
		  ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
										 // leave original code here
		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
	  }
	  
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
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

		  pid.UpdateError(cte);
		  steer_value = pid.TotalError();
		  pid.UpdateMse(cte);

		  // DEBUG
		  //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << "Loop Round: " <<loop_count<< std::endl;

		  if (steer_value > 1)
		  {
			  steer_value = 1;
		  }
		  else if (steer_value < -1)
		  {
			  steer_value = -1;
		  }
          
		  //update stuck count
		  if (speed < 0.05)
		  {
			  stuck_count++;
		  }
		  else
		  {
			  stuck_count = 0;
		  }
		  

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
		  if(useTwiddle)
		  {
			  if (loop_count >= MAX_STEPS || stuck_count >10)
			  {
				  //if stuck, peanalize based on how far you got
				  if (stuck_count > 10)
				  {
					  double penalty = 100000 * MAX_STEPS / loop_count;
					  pid.mse += penalty;
					  //std::cout << "penalty: " << penalty << " MSE: " << pid.mse << "Loop Round: " << loop_count << std::endl;
				  }
				  //reset the simulator

				  msg = "42[\"reset\",{}]"; // thanks to https://discussions.udacity.com/t/how-to-implement-twiddle-optimisation/279749/13
				  loop_count = 0;
				  stuck_count = 0;
				  pid.Twiddle();
			  }
			  else
			  {
				  loop_count++;
			  }
		  }

		  //std::cout << msg << std::endl;
#ifdef UWS_VCPKG
		  // code fixed for latest uWebSockets
		  ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		  // leave original code here
		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
		  
        }
		
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef UWS_VCPKG
		// code fixed for latest uWebSockets
		ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
		// leave original code here
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
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

#ifdef UWS_VCPKG
  // code fixed for latest uWebSockets
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
#else
  // leave original code here
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif
    //std::cout << "Connected!!!" << std::endl;
  });

#ifdef UWS_VCPKG
  // code fixed for latest uWebSockets
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
#else
  // leave original code here
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
#endif

#ifdef UWS_VCPKG
	  // code fixed for latest uWebSockets
	  ws->close();
#else
	  // leave original code here
	  ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1", port))
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
