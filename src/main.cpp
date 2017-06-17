#define _USE_MATH_DEFINES
#include <cmath> 
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

//For convenience
using json = nlohmann::json;

//For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.rfind("}]");
	if (found_null != string::npos) {
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

//Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

// Fit a polynomial.
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

	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
		string sdata = string(data).substr(0, length);
		//"42" at the start of the message means there's a websocket message event.
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();

				//We received new data
				if (event == "telemetry") {

					//Position of the waypoints from the simulator
					vector<double> ptsx = j[1]["ptsx"];
					vector<double> ptsy = j[1]["ptsy"];

					//Car position, angle and speed
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"];

					//Transform our waypoints so its easier to do the remaining calculations
					for (int i = 0; i < ptsx.size(); i++)
					{
						//Calcualte delta first, otherwise variable changes
						double dx = ptsx[i] - px;
						double dy = ptsy[i] - py;
						//Calculated transformed x and y position
						ptsx[i] = (dx) * cos(-psi) - (dy) * sin(-psi);
						ptsy[i] = (dx) * sin(-psi) + (dy) * cos(-psi);
					}

					//Conversion to VectorXd for polyfit
					Eigen::Map<Eigen::VectorXd> ptsx_transform(ptsx.data(), 6);
					Eigen::Map<Eigen::VectorXd> ptsy_transform(ptsy.data(), 6);

					//Calculate the coefficients from the 3rd order polynomial
					auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

					//Calculate cross track error at zero, y value is estimation of CTE
					double cte = polyeval(coeffs, 0);

					//Calculate error in orienation, is a simplification b/c psi and px is zero
					double epsi = -atan(coeffs[1]);

					//We create the state based on our transformations, given velocity and calc cte and epsi
					Eigen::VectorXd state(6);
					state << 0, 0, 0, v, cte, epsi;

					//Calculate errors in the future
					auto vars = mpc.Solve(state, coeffs);

					//Extract steer value and throttle
					double steering_value = vars[0];
					double throttle_value = vars[1];

					//Display the waypoints/reference line
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					//Display the line we want to follow / be close to in yellow
					for (int i = 0; i < 100; i = i + 3)
					{
						next_x_vals.push_back(i);
						next_y_vals.push_back(polyeval(coeffs, i));
					}

					//Display the MPC predicted trajectory 
					vector<double> mpc_x_vals;
					vector<double> mpc_y_vals;

					//Display our MPC predicted line in green, starts at 2
					for (int i = 2; i < vars.size(); i++)
					{
						//Every even index is a x
						if (i % 2 == 0)
							mpc_x_vals.push_back(vars[i]);
						//Every  odd index is a y
						else
							mpc_y_vals.push_back(vars[i]);
					}

					//Create our response package
					json msgJson;
					//Steering angle  needs to be normalized
					msgJson["steering_angle"] = -steering_value;
					//And throttle is already between -1 and +1
					msgJson["throttle"] = throttle_value;
					//Send MPC positions for trajectory
					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;
					//Send positions for ground-truth trajectory
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;
					//Incorporate then into a package
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";

					//Latency of 100ms to mimic real actuators
					this_thread::sleep_for(chrono::milliseconds(100));

					//Send it back to the simulator for control & visualisation
					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
		}
		else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
		char *message, size_t length) {
		ws->close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
