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
    virtual ~Obstacle(void) {};
};

class Map {
public:
    vector<double> wp_x;
    vector<double> wp_y;
    vector<double> wp_s;
    vector<double> wp_dx;
    vector<double> wp_dy;

    Map(vector<double> x, vector<double> y, vector<double> s, vector<double> dx, vector<double> dy);
    virtual ~Map(void) {};
};

class Vehicle {
private:
    double v_x;
    double v_y;
    double v_s;
    double v_d;
    double v_yaw;
    double v_speed;

    double target_speed;
    double max_acceleration;

    int lanes_available;
    int lane;
    int goal_lane;
    int goal_s;

    string state;
    Map *v_map;

public:
    int prev_size;
    vector<vector<double>> previous_path;
    vector<double> end_pos;

    vector<Obstacle> sensor_fusion;

    // contructor
    Vehicle(double x, double y, double s, double d, double yaw, double speed, Map *m);

    // destructor
    virtual ~Vehicle() {};

    string get_next_state(void);
    vector<vector<double>> generate_trajectory(void);
};

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif
