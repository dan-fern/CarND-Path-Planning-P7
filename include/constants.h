#ifndef CONSTANTS
#define CONSTANTS

#define VEHICLE_RADIUS 1.25              // meters

#define FOLLOW_DISTANCE 7.5              // distance to keep behind leading cars

#define SPEED_LIMIT 21.5                // m/s

#define VELOCITY_INCREMENT_LIMIT 0.125

#define TRACK_LENGTH 6945.554           // meters

#define PREVIOUS_PATH_POINTS_TO_KEEP 25

#define NUM_PATH_POINTS 50

#define PATH_DT 0.02                    // seconds

#define NUM_WAYPOINTS_BEHIND 5

#define NUM_WAYPOINTS_AHEAD 5

#define N_SAMPLES 20

#define DT 0.20                         // seconds

#define MAX_INSTANTANEOUS_ACCEL 10      // m/s/s

// cost function weights
#define COLLISION_COST_WEIGHT 99999
#define BUFFER_COST_WEIGHT 25
#define IN_LANE_BUFFER_COST_WEIGHT 1000
#define EFFICIENCY_COST_WEIGHT 10000
#define OUTER_LANE_COST_WEIGHT 50
#define SPEED_LIMIT_COST_WEIGHT 100

#endif
