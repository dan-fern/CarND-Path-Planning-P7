#include "cost.hpp"


double logistic( double x )
{
    /*
    Returns a value between 0 and 1 for x in the range[0, infinity] and
    -1 to 1 for x in the range[-infinity, infinity]. Useful for cost functions.
    */
    return 2.0 / (1 + exp(-x)) - 1.0;
}


double nearest_point(
        vector<double> s_traj,
        vector<double> d_traj,
        vector<vector<double>> prediction )
{
    /*
    Determines closest car comes to any other car throughout a trajectory
    */
    double nearest_distance = 999999;

    for( int i = 0; i < N_SAMPLES; i++ )
    {
        double current_distance = sqrt( pow( s_traj[i] - prediction[i][0], 2 ) +
                pow( d_traj[i] - prediction[i][1], 2 ) );

        if( current_distance < nearest_distance )
        {
            nearest_distance = current_distance;
        }
    }

    return nearest_distance;
}


double nearest_to_any_car(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions )
{
    /*
    Determines closest car comes to any other car throughout a trajectory
    */
    double nearest_distance = 999999;
    for( auto prediction : predictions )
    {
        double current_distance = nearest_point(
                s_traj,
                d_traj,
                prediction.second );

        if( current_distance < nearest_distance )
        {
            nearest_distance = current_distance;
        }
    }

    return nearest_distance;
}


double nearest_to_any_car_in_lane(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions)
{
    /*
    Determines closest car comes to other car throughout a trajectory in lane
    */
    double nearest_distance = 999999;

    for( auto prediction : predictions )
    {

        double ego_final_d = d_traj[ d_traj.size( ) - 1 ];
        int ego_lane = ego_final_d / 4;

        vector<vector<double>> pred_traj = prediction.second;

        double pred_final_d = pred_traj[ pred_traj.size( ) - 1 ][ 1 ];
        int pred_lane = pred_final_d / 4;

        if( ego_lane == pred_lane )
        {
            double current_distance = nearest_point(
                    s_traj,
                    d_traj,
                    prediction.second );

            if( current_distance < nearest_distance && current_distance < 120 )
            {
                nearest_distance = current_distance;
            }
        }
    }

    return nearest_distance;
}


vector<double> avg_trajectory_velocity( vector<double> traj )
{
    /*
    Return the average velocity between each pair of trajectories as a vector.
    */
    vector<double> velocities;

    for( int i = 1; i < traj.size( ); i++ )
    {
        velocities.push_back( ( traj[ i ] - traj[ i - 1 ] ) / DT );
    }

    return velocities;
}


double collision_cost(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions )
{
    /*
    Binary penalty for collisions.
    */
    double nearest_point = nearest_to_any_car( s_traj, d_traj, predictions );

    if( nearest_point < 2 * VEHICLE_RADIUS )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


double buffer_cost(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions )
{
    /*
    Penalizes proximity to other vehicles.
    */
    double nearest_point = nearest_to_any_car( s_traj, d_traj, predictions );

    return logistic( 2 * VEHICLE_RADIUS / nearest_point );
}


double buffer_cost_in_lane(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions )
{
    /*
    Penalizes proximity to other vehicles in lane.
    */
    double nearest = nearest_to_any_car_in_lane( s_traj, d_traj, predictions );

    return logistic( 2 * VEHICLE_RADIUS / nearest );
}


double exceeds_speed_limit_cost( vector<double> s_traj )
{
    /*
    Penalty if ds/dt for two points in trajectory is greater than SPEED_LIMIT
    */
    vector<double> s_dot_traj = avg_trajectory_velocity( s_traj );

    for( double s_dot : s_dot_traj )
    {
        if( s_dot > ( SPEED_LIMIT ) )
        {
            return 1;
        }
    }

    return 0;
}


double efficiency_cost( vector<double> s_traj )
{
    /*
    Penalizes scores with high variance.
    */
    vector<double> s_dot_traj = avg_trajectory_velocity( s_traj );

    double final_s_dot, total = 0;
    final_s_dot = s_dot_traj[ s_dot_traj.size( ) - 1 ];

    return logistic( ( SPEED_LIMIT - final_s_dot ) / SPEED_LIMIT );
}


double outer_lane_cost( vector<double> d_traj )
{
    /*
    Penalize not targeting the middle lane (d = 6)
    */
    double end_d = d_traj[ d_traj.size( ) - 1 ];

    return logistic( pow( end_d - 6, 2 ) );
}


double calculate_total_cost(
        vector<double> s_traj,
        vector<double> d_traj,
        map<int,vector<vector<double>>> predictions )
{
    double total_cost = 0;

    double cost_collision = collision_cost( s_traj, d_traj, predictions )
            * COLLISION_COST_WEIGHT;

    double cost_buffer = buffer_cost( s_traj, d_traj, predictions )
            * BUFFER_COST_WEIGHT;

    double cost_lane_buffer = buffer_cost_in_lane( s_traj, d_traj, predictions )
            * IN_LANE_BUFFER_COST_WEIGHT;

    double cost_efficiency = efficiency_cost( s_traj ) * EFFICIENCY_COST_WEIGHT;

    double cost_outer_lane = outer_lane_cost( d_traj ) * OUTER_LANE_COST_WEIGHT;

    double cost_speeding = exceeds_speed_limit_cost( s_traj )
            * SPEED_LIMIT_COST_WEIGHT;


    total_cost +=
            cost_collision
            + cost_buffer
            + cost_lane_buffer
            + cost_efficiency
            + cost_outer_lane
            + cost_speeding
            ;

    return total_cost;
}
