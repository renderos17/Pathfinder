#include "pathfinder/trajectory/coupled.h"
#include "pathfinder/math.h"

#include <stdio.h>

#define SPLINE(index) ((Pathfinder::Spline::Spline *)((uint8_t *)(_splines) + index*_spline_size))

int Pathfinder::Trajectory::Coupled::calculate(Pathfinder::Trajectory::CoupledSegment *segments_out, Pathfinder::Trajectory::CoupledSegment *last_segment, float time) {
    Pathfinder::Trajectory::CoupledSegment zero_seg;
    if (last_segment == nullptr) last_segment = &zero_seg;

    float cur_distance = last_segment->center.distance;

    if (cur_distance >= _total_distance) {
        // Path complete, nothing more
        // TODO set cleanup
        return 1; 
    }

    // Target Spline is our current, starting spline (i.e. we currently lay
    // between target_spline and target_spline+1)
    int target_spline = 0;
    float spline_dist_covered = 0, arc_length = 0;
    int done = 0;
    for (int i = 0; i < _spline_count; i++) {
        arc_length = SPLINE(i)->arc_length(_spline_samples);
        if (cur_distance - (spline_dist_covered + arc_length) < 0) {
            target_spline = i;
            break;
        }
        spline_dist_covered += arc_length;
    }
    //while (spline_dist_covered += SPLINE(target_spline)->arc_length(_spline_samples) < cur_distance) target_spline++;

    Pathfinder::Spline::Spline *spline = SPLINE(target_spline);
    float spline_distance = spline->arc_length(_spline_samples);
    // How far along the spline we are (0 to 1, our 'time' value for the spline, since they use an arbitrary time unit)
    float spline_progress = (cur_distance - spline_dist_covered) / spline_distance;

    Pathfinder::Spline::SplineCoord coord_center;
    spline->calculate(&coord_center, spline_progress);

    float   cosa = cos(coord_center.angle),
            sina = sin(coord_center.angle);

    // Cache the values from last_segment just incase segments_out is equal to last_segment
    float   dt = time - last_segment->center.time;
    float   last_angle = last_segment->center_2.angle;
    float   last_center_dist = last_segment->center.distance,
            last_left_dist  = last_segment->left.distance,
            last_right_dist = last_segment->right.distance;
    
    if (time == 0) {
        last_angle = coord_center.angle;
    }

    // Calculate the 2D Segments of the trajectory.
    segments_out->center_2.angle = coord_center.angle;
    segments_out->center_2.x = coord_center.x;
    segments_out->center_2.y = coord_center.y;

    segments_out->left_2.angle = coord_center.angle;
    segments_out->left_2.x = coord_center.x - (_wheelbase * sina);
    segments_out->left_2.y = coord_center.y + (_wheelbase * cosa);

    segments_out->right_2.angle = coord_center.angle;
    segments_out->right_2.x = coord_center.x + (_wheelbase * sina);
    segments_out->right_2.y = coord_center.y - (_wheelbase * cosa);

    // Start calculating the 1D Segments (distance / velocity / acceleration)
    segments_out->center.time = time;
    segments_out->left.time = time;
    segments_out->right.time = time;

    float angular_vel = (coord_center.angle - last_angle);
    if (angular_vel > M_PI) angular_vel = 2*M_PI - angular_vel;
    if (angular_vel < -M_PI) angular_vel = 2*M_PI + angular_vel;
    angular_vel /= dt;
    if (time == 0) {
        angular_vel = 0;    // Avoid divide by 0.
    }

    float tangential_speed = angular_vel * _wheelbase;
    float profile_max_vel = 3;     // TODO: Make this based on profile output at time.

    float vr, vl;
    if (angular_vel > 0) {
        // Counterclockwise, therefore right velocity is dominant. Set right to our maximum velocity (from profile),
        // solve for left velocity.
        vr = profile_max_vel;
        vl = (vr - tangential_speed);
    } else {
        // Clockwise, therefore left velocity is dominant. Set left to our maximum velocity (from profile),
        // solve for right velocity.
        vl = profile_max_vel;
        vr = vl + tangential_speed;
    }

    // The average speed between the two sides of the drivetrain will be the resultant 'effective' speed
    // at the center. 
    float center_speed = (vl + vr) / 2;
    segments_out->center.velocity = center_speed;
    segments_out->left.velocity = vl;
    segments_out->right.velocity = vr;

    segments_out->center.distance = last_center_dist + (center_speed*dt);
    segments_out->left.distance = last_left_dist + (vl*dt);
    segments_out->right.distance = last_right_dist + (vr*dt);

    return 0;
}