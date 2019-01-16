#ifndef COST
#define COST

#include <algorithm>
#include <map>

#include "constants.h"


using namespace std;


double logistic( double x );

double nearest_point(
        vector<double> s_traj,
        vector<double> d_traj,
        vector<vector<double>> prediction );

double nearest_to_any_car(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions );

double nearest_to_any_car_in_lane(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions );

vector<double> avg_trajectory_velocity( vector<double> traj );

double collision_cost(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions );

double buffer_cost(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions );

double buffer_cost_in_lane(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions );

double exceeds_speed_limit_cost( vector<double> s_traj );

double outer_lane_cost( vector<double> d_traj );

double calculate_total_cost(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions );

#endif
