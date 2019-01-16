#include "Eigen-3.3/Eigen/Dense"
#include "trajectory.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;


vector<double> interpolate(
        vector<double> pts_x,
        vector<double> pts_y,
        double interval,
        int output_size )
{
    /*
    uses the spline library to interpolate points connecting a series of x and y
    values output is output_size number of y values beginning at y[0] with
    specified fixed interval
    */

    if( pts_x.size( ) != pts_y.size( ) )
    {
        return { 0 };
    }

    tk::spline s;
    s.set_points( pts_x, pts_y );

    vector<double> output;
    for( int i = 0; i < output_size; i++ )
    {
        output.push_back( s( pts_x[0] + i * interval ) );
    }

    return output;
}


vector<double> interpolate(
        vector<double> pts_x,
        vector<double> pts_y,
        vector<double> eval_at_x )
{
    /*
    uses the spline library to interpolate points connecting a series of x and y
    values output is spline evaluated at each eval_at_x point
    */

    if( pts_x.size( ) != pts_y.size( ) )
    {
        return { 0 };
    }

    tk::spline s;
    s.set_points( pts_x, pts_y );

    vector<double> output;
    for( double x: eval_at_x )
    {
        output.push_back(s(x));
    }

    return output;
}


vector<double> get_traj_coeffs(
        vector<double> start,
        vector<double> end,
        double T )
{
    /*
    Jerk-minimizing trajectory that connects initial to final state in time T

    INPUT:
    start - vehicle start location given as a length three array corresponding
            to initial values of [s, s_dot, s_double_dot]
    end   - desired end state for vehicle in similar length three array.
    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT:
    array of length 6, with values corresponding to coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    */

    MatrixXd a( 3, 3 );

    double T2 =  T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;

    a <<    T3,    T4,     T5,
            3*T2,  4*T3,   5*T4,
            6*T,   12*T2,  20*T3;

    MatrixXd aInv = a.inverse( );

    VectorXd b( 3 );

    b << end[0] - ( start[0] + start[1] * T + 0.5 * start[2] * T2 ),
         end[1] - (            start[1]     +       start[2] * T ),
         end[2] - (                                 start[2] );

    VectorXd alpha = aInv * b;

    vector<double> output = {
            start[0],
            start[1],
            0.5 * start[2],
            alpha[0],
            alpha[1],
            alpha[2] };

    return output;
}
