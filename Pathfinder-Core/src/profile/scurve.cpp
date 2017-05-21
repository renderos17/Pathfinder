#include "pathfinder/profile/scurve.h"
#include "pathfinder/math.h"

void Pathfinder::Profile::SCurve::configure(float max_velocity, float max_acceleration, float jerk, float timescale, float tolerance) {
    _max_velocity = max_velocity;
    _max_acceleration = max_acceleration;
    _jerk = jerk;
    _timescale = timescale;
    _tolerance = tolerance;

    _velocity_profile.configure(_max_acceleration, _jerk, timescale, tolerance);
    _velocity_profile.setpoint(max_velocity);
}

uint8_t Pathfinder::Profile::SCurve::calculate_single(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time) {
    // Will get destroyed at end of scope. Placed here in the event last_segment is nullptr (i.e. start of generation),
    // we make an assumption of position 0, velocity 0, acceleration 0 and time 0.
    Pathfinder::Segment zero_seg = { 0,0,0,0 };
    if (last_segment == nullptr) last_segment = &zero_seg;
    uint8_t status = Pathfinder::Profile::STATUS_LEVEL;

    // Store these incase segment_out and last_segment point to the same memory address (in cases
    // where all generation is done on the fly)
    float   l_time = last_segment->time,
            l_dist = last_segment->distance,
            l_vel = last_segment->velocity,
            l_acc = last_segment->acceleration;
    
    Pathfinder::Segment     vel_seg_in = { l_time, l_vel, l_acc, 0 }, 
                            vel_seg = { 0, 0, 0, 0 };
    
    segment_out->time = time;
    float dt = time - l_time;

    if (fabs(l_dist - _setpoint) <= _tolerance) {
        // Drop all to 0 and return
        _jerk_out = 0;
        segment_out->acceleration = 0;
        segment_out->velocity = 0;
        segment_out->distance = l_dist;
        status = Pathfinder::Profile::STATUS_DONE;
        return status;
    }

    float jerk = 0;
    if (l_dist < _setpoint) {
        // Below setpoint
        jerk = _jerk;
    } else {
        // Above setpoint
        jerk = -_jerk;
    }

    int vpstatus = 0;
    float   max_a = (_setpoint > 0 ? _max_acceleration : -_max_acceleration);
    float   triangle_peak_time = sqrt(2*(l_vel/2)/jerk),
            saturation_time = (-max_a - l_acc)/(-jerk);

    float   dejerk_dist = l_vel * triangle_peak_time;
    if (fabs(saturation_time) < fabs(triangle_peak_time)) {
        float t0 = (-max_a - l_acc)/(-jerk);
        float t2 = max_a / jerk;

        float a0 = l_acc;
        float v0 = l_vel;
        float sixth = 1 / 6.0;

        float v1 = v0 + a0*t0 + 0.5*(-jerk)*t0*t0;
        float v2 = max_a*t2 + 0.5*(-jerk)*t2*t2;

        if (v1 < v2) t0 = 0;
        float t1 = (v1-v2)/max_a;
        if (t1 < 0) {
            t2 = t2 + t1;
            t1 = 0;
        }

        float s0 = v0*t0 + 0.5*a0*t0*t0 + sixth*(-jerk)*t0*t0*t0;
        float s1 = v1*t1 + 0.5*(-max_a)*t1*t1;
        float s2 = v2*t2 + 0.5*(-max_a)*t2*t2 + sixth*jerk*t2*t2*t2;

        dejerk_dist = s0+s1+s2;
    }

    float   dejerk_error = l_dist + dejerk_dist - _setpoint;

    // TODO: Unfortunately we can't use trapezoidal profiles for the velocity profile.
    // This is because we can't integrate the velocity the way we normally would, making it very
    // difficult to accurately generate a path.

    if (fabs(dejerk_error) <= _tolerance 
            || (_setpoint < 0 && dejerk_error < -_tolerance)
            || (_setpoint > 0 && dejerk_error > _tolerance)) {
        
        _velocity_profile._distance_integral = l_dist;
        _velocity_profile.setpoint(0);
        vpstatus = _velocity_profile.calculate(&vel_seg, &vel_seg_in, time);

        _jerk_out = vel_seg.acceleration;
        segment_out->acceleration = vel_seg.velocity;
        segment_out->velocity = vel_seg.distance;
        segment_out->distance = _velocity_profile._distance_integral;
        status = Pathfinder::Profile::STATUS_DECEL;
    } else if (fabs(l_vel) < _max_velocity - _tolerance) {
        _velocity_profile._distance_integral = l_dist;
        _velocity_profile.setpoint(_setpoint < 0 ? -_max_velocity : _max_velocity);
        vpstatus = _velocity_profile.calculate(&vel_seg, &vel_seg_in, time);

        _jerk_out = vel_seg.acceleration;
        segment_out->acceleration = vel_seg.velocity;
        segment_out->velocity = vel_seg.distance;
        segment_out->distance = _velocity_profile._distance_integral;
        status = Pathfinder::Profile::STATUS_ACCEL;
    }

    if (status == Pathfinder::Profile::STATUS_LEVEL) {
        // If nothing else, we have levelled out, hold steady velocity
        _jerk_out = 0;
        segment_out->acceleration = 0;
        segment_out->velocity = l_vel;
        segment_out->distance = l_dist + (l_vel * dt);
    }
    return status;
}