#ifndef TRAJECTORY
#define TRAJECTORY

#include "spline.h"


using namespace std;


vector<double> get_traj_coeffs(
        vector<double> start,
        vector<double> end,
        double T );

vector<double> interpolate(
        vector<double> pts_x,
        vector<double> pts_y,
        double interval,
        int output_size );

vector<double> interpolate(
        vector<double> pts_x,
        vector<double> pts_y,
        vector<double> eval_at_x );

#endif
