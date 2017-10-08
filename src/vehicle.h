#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

// For converting back and forth between radians and degrees.
static constexpr double pi() { return M_PI; }
static double deg2rad(double x) { return x * pi() / 180; }
static double rad2deg(double x) { return x * 180 / pi(); }

using namespace std;

class Vehicle;

class TrajectoryData {
public:
  vector<vector<double>> m_path;
  double m_speed;
  double m_max_accel;
  double m_rms_accel;
  bool m_collide;
  double m_collide_at;
  int m_proposed_lane;
  string m_state;

  Vehicle *m_car;

  TrajectoryData(Vehicle *car);
  virtual ~TrajectoryData(void);

  void update_trajectory_data(vector<vector<double>> path);
  void get_proposed_lane(string state, int current_lane);
};

class Obstacle {
public:
  double o_id;
  double o_x;
  double o_y;
  double o_vx;
  double o_vy;
  double o_s;
  double o_d;

  Obstacle(double id, double x, double y, double vx, double vy, double s, double d);
  virtual ~Obstacle(void);
};

class RouteMap {
public:
  vector<double> wp_x;
  vector<double> wp_y;
  vector<double> wp_s;
  vector<double> wp_dx;
  vector<double> wp_dy;

  RouteMap(vector<double> x, vector<double> y, vector<double> s, vector<double> dx, vector<double> dy);
  virtual ~RouteMap(void);
};

class Vehicle {
public:
  double v_x;
  double v_y;
  double v_s;
  double v_d;
  double v_yaw;
  double v_speed;

  double target_speed;
  double max_acceleration;

  int lanes_available;
  int m_lane;
  int goal_lane;
  int goal_s;

  string state;
  RouteMap *v_map;

  int prev_size;
  vector<vector<double>> previous_path;
  vector<double> end_pos;

  vector<Obstacle> sensor_fusion;

  Vehicle(double x, double y, double s, double d, double yaw, double speed, RouteMap *m);
  virtual ~Vehicle();

  vector<vector<double>> get_path(void);
  TrajectoryData* generate_trajectory(string state, double target_x, vector<double> wps, double vel);
  vector<vector<double>> evaluate_trajectory(vector<TrajectoryData*> data);
};

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif
