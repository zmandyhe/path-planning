#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

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
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}
	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	double heading = atan2( (map_y-y),(map_x-x) );
	double angle = abs(theta-heading);
	if(angle > pi()/4)
	{
		closestWaypoint++;
	}
	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance(0,0,proj_x,proj_y);
	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv"; 
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // start in lane 1
  int my_ref_lane = 1;
  // have a reference velocity to target
  double my_ref_vel = 0; // mph
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&my_ref_vel,&my_ref_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double my_x = j[1]["x"];
          double my_y = j[1]["y"];
          double my_s = j[1]["s"];
          double my_d = j[1]["d"];
          double my_yaw = j[1]["yaw"];
          double my_vel = j[1]["speed"];

          // Previous path data given returned from the simulator
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          int previous_path_size = previous_path_x.size();
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Sensor Fusion Data, a list of all other cars on the same side of the road
          auto other_cars = j[1]["sensor_fusion"];
            
          // if previous path does contain points, assume car is at
          // end position of previous path
          if (previous_path_size > 0) {my_s = end_path_s;}
            
          // initialize variable to stay on the current lane
          bool change_lanes_or_slow_down = false;
//           // stay at the current lane as much as it can
//           bool dont_go_left = true;
//           bool dont_go_right = true;
          // if my ego car is in the left-most or right-most lane
          // don't go left or go right
          bool dont_go_left = (my_ref_lane == 0);
          bool dont_go_right = (my_ref_lane == 2);
          short my_lane = ((short)floor(my_d/4));
          bool changing_lanes = (my_lane != my_ref_lane);
          // distance of leading vehicle ahead
          double min_dist_left = 999.0;
          double min_dist_here = 999.0;
          double min_dist_right = 999.0;
            
          // find unit normal vector at currernt position
          int other_waypoint_idx = NextWaypoint(my_x, my_y, my_yaw, map_waypoints_x, map_waypoints_y);
          double other_waypoint_dx = map_waypoints_dx[other_waypoint_idx];
          double other_waypoint_dy = map_waypoints_dy[other_waypoint_idx];
            
          // loop through other cars sharing the same side of the road
          // to understand their lanes, distance, intensions
          for (int i = 0; i < other_cars.size(); i++) {
            double other_x = other_cars[i][1];
            double other_y = other_cars[i][2];
            float other_d = other_cars[i][6];
            double other_vx = other_cars[i][3];
            double other_vy = other_cars[i][4];
            double other_v = sqrt(other_vx*other_vx + other_vy*other_vy);
            double other_s = other_cars[i][5];
            // calculate vd by dot product with d-vector
            double other_vd = other_vx*other_waypoint_dx + other_vy*other_waypoint_dy;
            // calculate vs by knowledge vs*vs + vd*vd = vx*vx + vy*vy
            double other_vs = sqrt(other_vx*other_vx + other_vy*other_vy - other_vd*other_vd);
            // project other cars lane position on 0.02s interval
            other_s += ((double)previous_path_size*.02*other_vs);
            // each lane is 4m wide, I use the left side of each lane for beachmark of that lane
            // 0: left-most lane,1:mid lane, 2: right-most lane
            short other_lane = ((short)floor(other_d/4));
            // project the other car direction (going left, going right)
            // project their lane to be, assuming car is in the center of lane
            bool other_is_going_right = (other_vd > 2.0);
            bool other_is_going_left  = (other_vd < -2.0);
            short other_direction     = other_is_going_right ? 1 : (other_is_going_left ? -1 : 0);
            short other_merging_lane  = other_lane + other_direction;
            // check if the car is in my lane or is entering my lane
            bool other_is_in_my_lane  = (other_lane == my_ref_lane) || (other_merging_lane == my_ref_lane);
            bool other_is_left  = (other_lane == my_ref_lane-1) || (other_merging_lane == my_ref_lane-1);
            bool other_is_right = (other_lane == my_ref_lane+1) || (other_merging_lane == my_ref_lane+1);
            // calculate the car's distance: if car is 30m ahead, 15m behind or closer than 10m
            double other_distance = other_s - my_s;
            bool other_is_ahead  = (other_distance > 0.0) && (other_distance < 30.0);
            bool other_is_close  = abs(other_s-my_s) < 10;
            bool other_is_behind = (other_distance < 0.0) && (other_distance > -15.0);
            // update all leading vehicles (<30m ahead of me) distance for all lanes
            if (other_distance > 0.0) {
              if (other_is_in_my_lane) {
                if (other_distance < min_dist_here) min_dist_here = other_distance;
              } else if (other_is_left) {
                if (other_distance < min_dist_left) min_dist_left = other_distance;
              } else if (other_is_right) {
                if (other_distance < min_dist_right) min_dist_right = other_distance;
              }
            }
            // check if car is slower
            bool other_is_slower = (other_v-my_ref_vel < 0);
            // if the leading vehicles is less than 30m ahead of me
            // then either change lane or slow down
            // follow the speed limit check
            change_lanes_or_slow_down = change_lanes_or_slow_down || (other_is_ahead && other_is_in_my_lane);
            // check if the car impedes my ability to swith lanes; if...
            // - closer than 30m ahead me and slower than me
            // - closer than 15m behind me and faster than me
            // - closer than 10m to me, it is an obstacle to change lane
            bool other_is_obstacle = ( (other_is_ahead && other_is_slower)
                                      || (other_is_behind && !other_is_slower)
                                      || other_is_close);
            // don't change lane if the car is an obstacle
            // don't change lance if the leading car in the target lane is closer than
            // the leading car in my current lane
            dont_go_left  = dont_go_left || (other_is_left && other_is_obstacle);
            dont_go_right = dont_go_right || (other_is_right && other_is_obstacle);
          }

//           // don't switch lanes if leading car in target lane is closer than
//           // leading car in current lane
          dont_go_left  = dont_go_left  || (min_dist_left < min_dist_here);
          dont_go_right = dont_go_right || (min_dist_right < min_dist_here);
          
          // if I have to change lane or slow down
          if (change_lanes_or_slow_down) {
            // if we are already changing lanes or cannot change 
            if (changing_lanes || (dont_go_left && dont_go_right) || my_ref_vel < 20) {
              // slow down instead (obeying max. accell. & velocity)
              if (my_ref_vel > 5)
                my_ref_vel -= .224;
            } else if (!dont_go_left) {
              // if left lane free, go there 
              my_ref_lane = (my_ref_lane - 1);
            } else if (!dont_go_right) {
              // if right lane free, go there
              my_ref_lane = (my_ref_lane + 1);
            }
          } else if (my_ref_vel < 49.5) {
            // speed up gradually but not over 50mph
            my_ref_vel += .224;
          }

          // create a list of widely spaced (x,y) waypoints, evenly spaced at
          // 30m, later we will interpolate these waypoints with a spline and 
          // fill it in with more points
          vector<double> control_x;
          vector<double> control_y;

          // reference x,y,yaw states: the state where we can actually control
          // the car right now; either the current state or the end of the
          // path calculated in the previous round (kept in order to smooth
          // the trajectory
          double my_ref_x = my_x;
          double my_ref_y = my_y;
          double ref_yaw = deg2rad(my_yaw);

          if (previous_path_size < 2) {
            // if previous size is almost empty, use the car as starting ref
            // but add one point behind it to make the path tangent to the car
            double my_prev_x = my_x - cos(my_yaw);
            double my_prev_y = my_y - sin(my_yaw);
            control_x.push_back(my_prev_x);
            control_x.push_back(my_x);
            control_y.push_back(my_prev_y);
            control_y.push_back(my_y);
          } else {
            // redefine reference state as previous path and point
            my_ref_x = previous_path_x[previous_path_size-1];
            my_ref_y = previous_path_y[previous_path_size-1];
            double my_prev_x = previous_path_x[previous_path_size-2];
            double my_prev_y = previous_path_y[previous_path_size-2];
            ref_yaw = atan2(my_ref_y-my_prev_y, my_ref_x-my_prev_x);
            // use two points that make the path tangent to the previous 
            // end point
            control_x.push_back(my_prev_x);
            control_x.push_back(my_ref_x);
            control_y.push_back(my_prev_y);
            control_y.push_back(my_ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          for (int spacing = 30; spacing <= 90; spacing += 30) {
            vector<double> controlpoint = getXY(my_s+spacing, (2+4*my_ref_lane), 
                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
            control_x.push_back(controlpoint[0]);
            control_y.push_back(controlpoint[1]);
          }

          // transform controlpoints to car coordinate starting from 
          // reference position
          vector<double> control_x_car;
          vector<double> control_y_car;

          for (int i = 0; i < control_x.size(); i++) {
            // shift
            double x_shifted = control_x[i]-my_ref_x;
            double y_shifted = control_y[i]-my_ref_y;
            // rotate
            control_x_car.push_back(x_shifted*cos(0-ref_yaw)-y_shifted*sin(0-ref_yaw));
            control_y_car.push_back(x_shifted*sin(0-ref_yaw)+y_shifted*cos(0-ref_yaw));
          }

          // create a spline & set (x,y) points to the spline
          tk::spline spline_car;
          spline_car.set_points(control_x_car, control_y_car);

          // sample (x,y) points from spline
          vector<double> my_next_x;
          vector<double> my_next_y;

          // start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); i++) {
            my_next_x.push_back(previous_path_x[i]);
            my_next_y.push_back(previous_path_y[i]);
          }

          // calculate with which distance to interpolate along the spline 
          // so we travel at our desired reference velocity
          // path will extend 30m ahead in x direction
          double target_x_car = 30.0;
          // get y value for that distance
          double target_y_car = spline_car(target_x_car);
          // calculate euclidean distance travelled to that point
          double target_dist = sqrt(target_x_car*target_x_car + target_y_car*target_y_car);
          // calculate spacing of points needed to find reference velocity
          double num_points = target_dist / (.02*my_ref_vel/2.24);

          // fill up the rest of our path planner so we have 50 points
          double recent_x_car = 0;
          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
            double x_car = recent_x_car + target_x_car/num_points;
            double y_car = spline_car(x_car);
            recent_x_car = x_car;
            // transform car coordinates to world coordinates
            // rotate
            double x = (x_car*cos(ref_yaw)-y_car*sin(ref_yaw));
            double y = (x_car*sin(ref_yaw)+y_car*cos(ref_yaw));
            // shift
            x += my_ref_x;
            y += my_ref_y;
            // add to return value
            my_next_x.push_back(x);
            my_next_y.push_back(y);
          }

          json msgJson;
          msgJson["next_x"] = my_next_x;
          msgJson["next_y"] = my_next_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
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
