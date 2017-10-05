#include "vehicle.h"
#include "spline.h"

double ref_vel = 0.0;
int g_lane = 1;

Map::Map(vector<double> x, vector<double> y, vector<double> s, vector<double> dx, vector<double> dy) {
  wp_x = x;
  wp_y = y;
  wp_s = s;
  wp_dx = dx;
  wp_dy = dy;
}

Obstacle::Obstacle(double id, double x, double y, double vx, double vy, double s, double d) {
  o_id = id;
  o_x = x;
  o_y = y;
  o_vx = vx;
  o_vy = vy;
  o_s = s;
  o_d = d;
}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed, Map *m) {
  v_x = x;
  v_y = y;
  v_s = s;
  v_d = d;
  v_yaw = yaw;
  v_speed = speed;
  v_map = m;

  lane = g_lane;
}

string Vehicle::get_next_state(void) {
}

vector<vector<double>> Vehicle::generate_trajectory(void) {
  double car_s;
  vector<vector<double>> next_vals;

  car_s = v_s;

  // if there are waypoints from previous path then start from end of that
  if (prev_size > 2) {
    car_s = end_pos[0];
  }

  bool too_close = false;

  for (Obstacle obj: sensor_fusion) {
    // car is in my lane
    float d = obj.o_d;
    if (d < (2+4*lane+2) && d > (2+4*lane-2)) {
      double vx = obj.o_vx;
      double vy = obj.o_vy;

      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = obj.o_s;

      // if we are using previous path points, it means this car is also not at
      // at the end of path, calculate where would this car be at then end of path
      check_car_s += ((double)prev_size*0.02*check_speed);
      if ((check_car_s > car_s) && ((check_car_s-car_s)<30)) {
        //ref_vel = 29.5;
        too_close = true;
        if (lane > 0) {
          lane = 0;
          g_lane = lane;
        }
      }
    }
  }

  if (too_close) {
    ref_vel -= 0.224;
  } else if (ref_vel < 49.5) {
    ref_vel += 0.224;
  }

  // create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // Later we will interpolate these with a spline and fill it in with more points that control speed
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x,y,yaw states
  // either we will reference the starting point as where the car is or at the previous paths end point
  double ref_x = v_x;
  double ref_y = v_y;
  double ref_yaw = deg2rad(v_yaw);

  if (prev_size < 2) {
    double prev_car_x = v_x - cos(v_yaw);
    double prev_car_y = v_y - sin(v_yaw);
    ptsx.push_back(prev_car_x);
    ptsx.push_back(v_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(v_y);

  } else {
    ref_x = previous_path[prev_size-1][0];
    ref_y = previous_path[prev_size-1][1];

    double ref_x_prev = previous_path[prev_size-2][0];
    double ref_y_prev = previous_path[prev_size-2][1];

    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = getXY(v_s+30, (2+4*lane), v_map->wp_s, v_map->wp_x, v_map->wp_y);
  vector<double> next_wp1 = getXY(v_s+60, (2+4*lane), v_map->wp_s, v_map->wp_x, v_map->wp_y);
  vector<double> next_wp2 = getXY(v_s+90, (2+4*lane), v_map->wp_s, v_map->wp_x, v_map->wp_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  //shift car reference angle to 0 degrees
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y * cos(0-ref_yaw));
  }

  // create a spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx,ptsy);

  // start with previous pending points
  for (vector<double> state : previous_path) {
    next_vals.push_back(state);
  }

  //calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

  double x_add_on = 0;
  // distance = N * 0.02 * ref_vel
  // N = number of points
  // 0.02 = simulator car visits each point in 20ms
  // ref_vel = Target velocity
  double N = (target_dist/(0.02*ref_vel/2.24));
  double step_size = target_x/N;

  for (int i = 0; i <= 50 - previous_path.size(); i++) {
    double x_point = x_add_on+step_size;
    double y_point = s(x_point);
    vector<double> state;

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after ratating it earlier
    x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    state.push_back(x_point);
    state.push_back(y_point);

    next_vals.push_back(state);
  }
  return next_vals;
}

