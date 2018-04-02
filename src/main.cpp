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
constexpr double pi() {
    return M_PI;
}
double deg2rad(double x) {
    return x * pi() / 180;
}
double rad2deg(double x) {
    return x * 180 / pi();
}

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


/*

UL : Have removed - as potentially in-efficient

// Evaluate a polynomial.

double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}
*/

//
// UL : This is my preffered way to evaluate polynomials. May hav nan problems
//

double polyeval(Eigen::VectorXd coeffs, double x) {
    int n = coeffs.size() ;

    double result = coeffs[--n] ;
    while ( n  )
        result = ( result * x ) + coeffs[--n] ;

    return result ;
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
    return  Q.solve(yvals);

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
        //cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {

                    // j[1] is the data JSON object


                    vector<double> ptsx = j[1]["ptsx"] ; // waypoints x - global world co-ordinates
                    vector<double> ptsy = j[1]["ptsy"] ; // waypoints y
                    double px           = j[1]["x"];     // current x
                    double py           = j[1]["y"];     // current y
                    double psi          = j[1]["psi"];   // current psi [ ]
                    double v            = j[1]["speed"]; // speed in mph


                    // UL : The DATA.md file states spedouble delta = j[1]["steering_angle"]; is in MPH
                    // the tips-and-tricks state the x,y units are in meters
                    // so in theory should convert to m/s-1 but that dosent work so one or other statement is wrong
                    //

                    //
                    // Calculate steering angle and throttle using MPC.
                    //

                    //
                    // UL : First - we transform waypoints from global to local space ..
                    // we do this by rotating the global points around the car center , clockwise by
                    // it's current heading.
                    //
                    // This makes the car heading along the +ve x axis , and the +ve y axis is on the left.
                    //



                    Eigen::VectorXd way_x ( ptsx.size());
                    Eigen::VectorXd way_y ( ptsy.size());

                    // precompute cos/sin

                    double cos_psi = cos( psi ) ;
                    double sin_psi = sin( psi ) ;

                    for ( size_t i = 0 ; i < ptsx.size() ; i++ ) {

                        // translate point to car origin
                        double dx = ( ptsx[i] - px ) ;
                        double dy = ( ptsy[i] - py ) ;

                        // rotate clockwise by psi - which is same as ccw by -psi

                        way_x[i] = dx * cos_psi + dy * sin_psi ;
                        way_y[i] = dy * cos_psi - dx * sin_psi ;


                    }


                    //
                    // UL : fit the poly to the car's now relative waypoints.
                    //

                    auto coeffs = polyfit( way_x, way_y, 3) ;



                    // UL : calculate an approximate cross track error
                    //
                    // If we evaluate the above waypoint polynomial function  at x = 0
                    // the resulting value is the distance from (car to track).
                    // Since x = 0 this can be computed simply as coeffs[0]

                    double cte  = coeffs[0]		     ; // coeffs[3]*(0*0*0) + coeffs[2]*(0*0) + coeffs[1]*(0) + coeffs[0] = coeffs[0]

                    // UL : calculate an approx heading error.
                    //
                    // We calculate the gradient of the polynomial at x = 0
                    // if we take the inverse tan - we get the angle of the tangent at x = 0
                    // since we know we are currently heading at psi = 0 , the errors is simply -atan( coeffs[1] )

                    double epsi = -atan( coeffs[1] ) ; // -atan( 3*coeffs[3]*(0*0) + 2*coeffs[2]*(0) + coeffs[1] )  = -atan( )

                    // UL : account for latency.
                    //
                    // The simulate is given the required acctuators 100ms + computation time too late.
                    // We can overcome this to some extent by instead of using the starting state of the car.
                    // we predict forward where the car is likely to be in 100ms time , and start are predictions
                    // from there .
                    //
                    // we use the simple kinematic model - that the solver uses to work out a forward state.
                    //


                    // UL : current steering angle and throttle - usefull
                    double delta = j[1]["steering_angle"];
                    double a = j[1]["throttle"];

                    // UL : set the fixed term latency

                    const double latency = 0.1 ; // the forward latency in this case 100ms .

                    // UL : compute the forward state variables

                    double fwd_px   = v * latency ;
                    double fwd_py   = 0.0;
                    double fwd_psi  = v * -delta / Lf * latency ;
                    double fwd_v    = v + a * latency ;
                    double fwd_cte  = cte + v * sin(epsi) * latency ;
                    double fwd_epsi = epsi + v * -delta / Lf * latency ;


                    // UL : setup the state vector

                    Eigen::VectorXd state(6);

                    // state << 0 , 0 , 0 , v , cte , epsi ;
                    state << fwd_px, fwd_py, fwd_psi, fwd_v, fwd_cte, fwd_epsi ;

                    // UL : try to solve
                    auto result = mpc.Solve( state, coeffs ) ;


                    // UL : the solver gives us the steering / throttle values to use
                    // these values are what the solver thinks would give the least cost.

                    double steer_value    = result[0]/( deg2rad(25) * Lf ) ;
                    double throttle_value = result[1] ;

                    // UL : for debuging
                    // std::cout << psi << "," << v << "," << cte << "," << epsi << " {" << steer_value << "," << throttle_value << "}" << std::endl ;


                    json msgJson;


                    msgJson["steering_angle"] = steer_value  ;
                    msgJson["throttle"] = throttle_value ;

                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    for ( size_t i = 2 ; i < result.size() ; i+=2 ) {
                        mpc_x_vals.push_back( result[i+0] ) ;
                        mpc_y_vals.push_back( result[i+1] ) ;
                    }

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    //
                    // we use the polynomial to evaluate the waypoint line at various intervals in the future ( ie x > 0 )
                    //


                    for ( double x = 1.0 ; x < 50.0 ; x+=5.0  ) {
                        next_x_vals.push_back( x ) ;
                        next_y_vals.push_back( polyeval( coeffs, x )) ;
                    }



                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // std::cout << msg << std::endl;

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
