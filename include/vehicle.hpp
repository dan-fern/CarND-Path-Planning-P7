#ifndef VEHICLE
#define VEHICLE

#include <map>

#include "constants.h"
#include "trajectory.hpp"


using namespace std;


class Vehicle
{

public:

    double s;
    double s_d;
    double s_dd;
    double d;
    double d_d;
    double d_dd;
    string state;
    string prior_state;
    vector<string> available_states;
    vector<double> s_traj_coeffs, d_traj_coeffs;

    /**
    * Constructors
    */
    Vehicle( );

    Vehicle(
            double s,
            double s_d,
            double s_dd,
            double d,
            double d_d,
            double d_dd );

    /**
    * Destructor
    */
    virtual ~Vehicle( );

    void update_available_moves( bool car_to_left, bool car_to_right );

    vector<vector<double>> get_move_target(
            string state,
            map<int, vector<vector<double>>> predictions,
            double duration,
            bool car_just_ahead );

    vector<double> get_lead_car_data(
            int target_lane,
            map<int, vector<vector<double>>> predictions,
            double duration );

    vector<vector<double>> generate_target_traj(
            vector<vector<double>> perturbed_target,
            double duration );

    vector<double> differentiate_coeffs( vector<double> coeffs );

    double evaluate_coeffs_at_time( vector<double> coeffs, double time );

    vector<vector<double>> generate_predictions(
            double traj_start_time,
            double duration );

};

#endif
