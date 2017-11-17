#include "vehicle.h"
#include "spline.h"

double ref_vel = 0.0;
int g_lane = 1;

TrajectoryData::TrajectoryData(Vehicle *car) {
  m_car = car;
}

TrajectoryData::~TrajectoryData(void) {}

RouteMap::RouteMap(vector<double> x, vector<double> y, vector<double> s, vector<double> dx, vector<double> dy) {
  wp_x = x;
  wp_y = y;
  wp_s = s;
  wp_dx = dx;
  wp_dy = dy;
}

RouteMap::~RouteMap(void) {}

Obstacle::Obstacle(double id, double x, double y, double vx, double vy, double s, double d) {
  o_id = id;
  o_x = x;
  o_y = y;
  o_vx = vx;
  o_vy = vy;
  o_s = s;
  o_d = d;
}

Obstacle::~Obstacle(void) {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed, RouteMap *m) {
  v_x = x;
  v_y = y;
  v_s = s;
  v_d = d;
  v_yaw = yaw;
  v_speed = speed;
  v_map = m;

  m_lane = g_lane;
}

Vehicle::~Vehicle(void) {}

void TrajectoryData::update_trajectory_data(vector<vector<double>> path) {
  double car_s;
  vector<double> accels;
  vector<double> jerks;

  m_path = path;
  car_s = m_car->v_s;
  m_collide = false;

  // if there are waypoints from previous path then start from end of that
  //if (m_car->prev_size > 2) {
  //  car_s = m_car->end_pos[0];
  //}

  double car_s1;
  double car_s2;

  for (Obstacle obj: m_car->sensor_fusion) {
    // car is in my lane
    float d = obj.o_d;
    if (d < (2+4*m_proposed_lane+2) && d > (2+4*m_proposed_lane-2)) {
      double vx = obj.o_vx;
      double vy = obj.o_vy;

      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = obj.o_s;

      // if we are using previous path points, it means this car is also not at
      // at the end of path, calculate where would this car be at the end of path
      check_car_s += ((double)m_car->prev_size*0.02*check_speed);

      for (int i = 0; i < path.size(); i++) {
        double ego_s;
        double other_s;

        ego_s = car_s + (i * 0.02 * m_car->v_speed);
        other_s = check_car_s + (i * 0.02 * check_speed);
        if ((other_s > ego_s) && ((other_s-ego_s)<50)) {
          m_collide = true;
        }
      }
    }
  }
}

string last_state = "KL";

vector<vector<double>> Vehicle::evaluate_trajectory(vector<TrajectoryData*> data) {
  TrajectoryData* next_path = NULL;

  for (TrajectoryData *test : data) {
    if (!test->m_collide && test->m_speed < 49.5) {
      //cout << "Selected state = " << test->m_state << ", path len = " << test->m_path.size() << endl;
      next_path = test;
      break;
//      return test->m_path;
    }
  }

  if (next_path == NULL) {
    goto not_found;
  }

  if (next_path->m_state == "LCL" && next_path->m_speed > 30) {
    for (TrajectoryData *test : data) {
      if (test->m_state == "PLCL") {
        next_path = test;
        break;
      }
    }
  } else if (next_path->m_state == "LCR" && next_path->m_speed > 30) {
    for (TrajectoryData *test : data) {
      if (test->m_state == "PLCR") {
        next_path = test;
        break;
      }
    }
  }

not_found:
  if (next_path == NULL) {
    cout << "ERRROR no valid path found" << endl;
    for (TrajectoryData *test : data) {
      cout << test->m_state << ": speed = " << test->m_speed << ", collide = " << test->m_collide << ", lane = " << test->m_proposed_lane << endl;
    }
    next_path = data[4];
  }

  if (next_path->m_state == "LCR" || next_path->m_state == "LCL" || last_state == "LCL" || last_state == "LCR") {
    cout << next_path->m_state << ": speed = " << next_path->m_speed << ", collide = " << next_path->m_collide << ", lane = " << next_path->m_proposed_lane << endl;
  }

  ref_vel = next_path->m_speed;
  g_lane = next_path->m_proposed_lane;

  last_state = next_path->m_state;

  return next_path->m_path;
}

// calculate proposed lane depending on current lane and proposed state
void TrajectoryData::get_proposed_lane(string state, int current_lane) {

  if (state == "LCL" && current_lane > 0) {
    m_proposed_lane = current_lane - 1;
  } else if (state == "LCR" && current_lane < 2) {
    m_proposed_lane = current_lane + 1;
  } else {
    m_proposed_lane = current_lane;
  }
}

TrajectoryData* Vehicle::generate_trajectory(string state, double target_x, vector<double>wps, double vel) {

  vector<vector<double>> next_vals;
  TrajectoryData *t_data = new TrajectoryData(this);

  t_data->get_proposed_lane(state, m_lane);
  t_data->m_speed = vel;
  t_data->m_state = state;

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

  for (double wp : wps) {
    vector<double> next_wp = getXY(v_s+wp, (2+4*t_data->m_proposed_lane), v_map->wp_s, v_map->wp_x, v_map->wp_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
  }

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
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

  double x_add_on = 0;
  // distance = N * 0.02 * vel
  // N = number of points
  // 0.02 = simulator car visits each point in 20ms
  // vel = Target velocity
  double N = (target_dist/(0.02*vel/2.24));
  double step_size = target_x/N;

  for (int i = 0; i <= 50 - previous_path.size(); i++) {
    double x_point = x_add_on+step_size;
    double y_point = s(x_point);
    vector<double> state;

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating it earlier
    x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    state.push_back(x_point);
    state.push_back(y_point);

    next_vals.push_back(state);
  }

  t_data->update_trajectory_data(next_vals);

  return t_data;
}

vector<vector<double>> Vehicle::get_path(void) {
  TrajectoryData *path;
  vector<TrajectoryData*> proposed_paths;
  vector<string> states = {"KL", "LCL", "LCR", "PLCL", "PLCR"};
  vector<vector<double>> valid_path;
  vector<double> wps1 = {30,60,90};
  vector<double> wps2 = {60,75,90};

  path = this->generate_trajectory(states[0], 30.0, wps1, ref_vel+0.224);
  proposed_paths.push_back(path);
  path = this->generate_trajectory(states[0], 30.0, wps1, ref_vel);
  proposed_paths.push_back(path);
  path = this->generate_trajectory(states[1], 30.0, wps2, ref_vel);
  proposed_paths.push_back(path);
  path = this->generate_trajectory(states[2], 30.0, wps2, ref_vel);
  proposed_paths.push_back(path);
  path = this->generate_trajectory(states[0], 30.0, wps1, ref_vel-0.224);
  proposed_paths.push_back(path);
  path = this->generate_trajectory(states[3], 30.0, wps1, ref_vel-0.224);
  proposed_paths.push_back(path);
  path = this->generate_trajectory(states[4], 30.0, wps1, ref_vel-0.224);
  proposed_paths.push_back(path);

  vector<vector<double>> next_vals = this->evaluate_trajectory(proposed_paths);

  for (int i = 0; i < next_vals.size(); i++) {
    valid_path.push_back(next_vals[i]);
  }

  for (int i = 0; i < proposed_paths.size(); i++) {
    path = proposed_paths[i];
    free(path);
  }

  return valid_path;
}
