#include "vehicle.hpp"

Vehicle::Vehicle( )
{

}


Vehicle::Vehicle(
        double s,
        double s_d,
        double s_dd,
        double d,
        double d_d,
        double d_dd )
{
    /**
    * Initializes Vehicle
    */

    this->s    = s;         // s position
    this->s_d  = s_d;       // s dot - velocity in s
    this->s_dd = s_dd;      // s dot-dot - acceleration in s
    this->d    = d;         // d position
    this->d_d  = d_d;       // d dot - velocity in d
    this->d_dd = d_dd;      // d dot-dot - acceleration in d
    state = "CS";
    prior_state = "KL";

}

Vehicle::~Vehicle( )
{

}


void Vehicle::update_available_moves( bool car_to_left, bool car_to_right )
{
    /**
    Updates the available actions based on the current state:
    "KL" - Keep Lane and slow down (if needed)
    "LCL" or "LCR" - Lane Change Left / Right
    */
    string previous_state = this->prior_state;

    this->available_states = {"KL"};
    if( this->d > 4 && !car_to_left && this->prior_state != "LCR" )
    {
        this->available_states.push_back( "LCL" );
    }
    if( this->d < 8 && !car_to_right && this->prior_state != "LCL" )
    {
        this->available_states.push_back( "LCR" );
    }
}


vector<vector<double>> Vehicle::get_move_target(
        string state,
        map<int, vector<vector<double>>> predictions,
        double duration,
        bool car_just_ahead )
{
    /*
    Returns s_target and d_target in a single nested vector with
    [s, s_dot, and s_ddot] and [d, d_dot, and d_dot].
    If there is no leading car, ego car will make up PERCENT_V_DIFF_TO_MAKE_UP
    of the difference between current velocity and target velocity.
    If leading car is found set target s to FOLLOW_DISTANCE and target s_dot
    to leading car's s_dot based on prediction.
    */
    int target_lane, current_lane = this->d / 4;
    double target_d;

    // target lateral displacement depends on state, say lateral velocity : 0
    // lateral acceleration : 0
    double target_d_d = 0;
    double target_d_dd = 0;

    // longitudinal velocity : current velocity + max allowed accel * duration
    // longitudinal acceleration : zero ?
    double target_s_d = min( this->s_d + MAX_INSTANTANEOUS_ACCEL / 4 * duration,
            SPEED_LIMIT );
    double target_s_dd = 0;

    // longitudinal displacement : difference in current/target velocity times
    // trajectory duration
    double target_s = this->s + ( this->s_d + target_s_d ) / 2 * duration;

    vector<double> lead_car_s_sdot;

    if( state.compare( "KL" ) == 0 )
    {
        target_d = (double)current_lane * 4 + 2;
        target_lane = target_d / 4;
    }
    else if( state.compare( "LCL" ) == 0 )
    {
        target_d = ( (double)current_lane - 1 ) * 4 + 2;
        target_lane = target_d / 4;
    }
    else if( state.compare( "LCR" ) == 0 )
    {
        target_d = ( (double)current_lane + 1 ) * 4 + 2;
        target_lane = target_d / 4;
    }

    // replace target_s variables if there is a leading vehicle close enough
    lead_car_s_sdot = get_lead_car_data( target_lane, predictions, duration );

    double lead_car_s = lead_car_s_sdot[0];

    if( lead_car_s - target_s < FOLLOW_DISTANCE && lead_car_s > this->s )
    {
        target_s_d = lead_car_s_sdot[1];

        if( fabs( lead_car_s - target_s ) < 0.5 * FOLLOW_DISTANCE )
        {
            // slow down if too close
            target_s_d -= 2.5;
        }

        target_s = lead_car_s - FOLLOW_DISTANCE;
    }

    // emergency brake
    if( car_just_ahead )
    {
        target_s_d = 0.0;
    }

    return { { target_s, target_s_d, target_s_dd },
            { target_d, target_d_d, target_d_dd } };
}


vector<double> Vehicle::get_lead_car_data(
        int target_lane,
        map<int, vector<vector<double>>> predictions,
        double duration )
{
    /*
    returns s and s_dot for the nearest leading vehicle in target lane assuming
    the vehicle will keep its lane and velocity, it will return the end position
    and velocity (based on difference between last two positions)
    */
    double nearest_lead_car_speed = 0, nearest_leading_vehicle_distance = 99999;

    for( auto prediction : predictions )
    {
        vector<vector<double>> pred_traj = prediction.second;

        int pred_lane = pred_traj[0][1] / 4;

        if( pred_lane == target_lane )
        {
            double start_s = pred_traj[0][0];
            double predicted_end_s = pred_traj[pred_traj.size()-1][0];
            double next_to_last_s = pred_traj[pred_traj.size()-2][0];

            double dt = duration / N_SAMPLES;

            double predicted_s_dot = ( predicted_end_s - next_to_last_s ) / dt;

            if ( predicted_end_s < nearest_leading_vehicle_distance &&
                    start_s > this->s )
            {
                nearest_leading_vehicle_distance = predicted_end_s;
                nearest_lead_car_speed = predicted_s_dot;
            }
        }
    }

    return { nearest_leading_vehicle_distance, nearest_lead_car_speed };
}


vector<vector<double>> Vehicle::generate_target_traj(
        vector<vector<double>> target,
        double duration )
{
    /*
    takes a target {{s, s_dot, s_ddot}, {d, d_dot, d_ddot}} and returns a
    jerk-minimized trajectory connecting current state (s and d) to target
    state in a list of s points and a list of d points
    */
    vector<double> current_s = { this->s, this->s_d, this->s_dd };
    vector<double> current_d = { this->d, this->d_d, this->d_dd };

    vector<double> target_s = target[0];
    vector<double> target_d = target[1];

    vector<double> s_traj;
    vector<double> d_traj;

    // determine coefficients of optimal trajectory
    this->s_traj_coeffs = get_traj_coeffs( current_s, target_s, duration );
    this->d_traj_coeffs = get_traj_coeffs( current_d, target_d, duration );

    // populate s and t trajectories at each time step
    for( int i = 0; i < N_SAMPLES; i++ )
    {
        double t = i * duration / N_SAMPLES, s_val = 0, d_val = 0;

        for ( int j = 0; j < s_traj_coeffs.size( ); j++ )
        {
            s_val += this->s_traj_coeffs[j] * pow( t, j );
            d_val += this->d_traj_coeffs[j] * pow( t, j );
        }

    s_traj.push_back( s_val );
    d_traj.push_back( d_val );

    }

    return { s_traj, d_traj };
}


vector<double> Vehicle::differentiate_coeffs( vector<double> coeffs )
{
    vector<double> differentiated_coeffs;

    for( int i = 1; i < coeffs.size( ); i++ )
    {
        differentiated_coeffs.push_back( i * coeffs[i] );
    }

    return differentiated_coeffs;
}


double Vehicle::evaluate_coeffs_at_time( vector<double> coeffs, double time )
{
    double evaluated_coeffs = 0;

    for( int i = 0; i < coeffs.size(); i++ )
    {
        evaluated_coeffs += coeffs[i] * pow( time, i );
    }

    return evaluated_coeffs;
}


vector<vector<double>> Vehicle::generate_predictions(
        double traj_start_time,
        double duration)
{
    /*
    Generates a list of predicted s and d positions for constant-speed vehicles
    Because ego car trajectory is considered from end of previous path, we
    should also consider the trajectories of other cars starting at that time.
    */

    vector<vector<double>> predictions;

    for( int i = 0; i < N_SAMPLES; i++ )
    {
        double t = traj_start_time + ( i * duration / N_SAMPLES );
        double new_s = this->s + this->s_d * t;

        vector<double> s_and_d = { new_s, this->d };

        predictions.push_back( s_and_d );
    }

    return predictions;
}
