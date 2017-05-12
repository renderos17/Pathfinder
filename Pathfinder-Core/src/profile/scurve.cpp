#include "pathfinder/profile/scurve.h"
#include "pathfinder/math.h"

void Pathfinder::Profile::SCurve::configure(float max_velocity, float max_acceleration, float jerk, float tolerance) {
    _max_velocity = max_velocity;
    _max_acceleration = max_acceleration;
    _jerk = jerk;
    _tolerance = tolerance;

    _velocity_profile.configure(_max_acceleration, _jerk, tolerance);
    _velocity_profile.setpoint(max_velocity);
}

uint8_t Pathfinder::Profile::SCurve::calculate(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time) {
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

    // TODO: Find when to de-jerk
    // We could legit just use another trapezoidal profile for this, but give it values of velocity instead of distance

    float jerk = 0;
    if (l_dist < _setpoint) {
        // Below setpoint
        jerk = _jerk;
    } else {
        // Above setpoint
        jerk = -_jerk;
    }

    int vpstatus = 0;

    // float   dejerk_time = (-l_acc - sqrt(l_acc*l_acc-2*(-jerk)*l_vel))/(-jerk),
            // dejerk_dist = l_vel*dejerk_time + (0.5*l_acc + (1/6.0)*-jerk*dejerk_time)*dejerk_time*dejerk_time;
    // printf("%.2f %.2f %.2f\n", dejerk_time, dejerk_dist, time);

    // TODO: Handle case where the acceleration levels out
    float   max_a = (_setpoint > 0 ? _max_acceleration : -_max_acceleration);
    float   jerk_switch_time = sqrt(2*(l_vel/2)/jerk),
            jerk_accel_time = max_a / jerk;

    float   dejerk_dist = l_vel * jerk_switch_time;
    if (fabs(jerk_accel_time) < fabs(jerk_switch_time)) {
        float t0 = jerk_accel_time;
        float v0 = l_vel + 0.5*-jerk*t0*t0;
        float t1 = (2*v0-l_vel)/max_a;
        float v1 = l_vel - v0;
        float sixth = (1/6.0);

        float s0 = l_vel*t0 + sixth*-jerk*t0*t0*t0;
        float s1 = v0*t1 + 0.5*(-max_a)*t1*t1;
        float s2 = v1*t0*t0 + 0.5*(-max_a)*t0*t0 + sixth*jerk*t0*t0*t0;

        dejerk_dist = s0+s1+s2;
    }

    float   dejerk_error = l_dist + dejerk_dist - _setpoint;

    if (fabs(dejerk_error) <= _tolerance 
            || (_setpoint < 0 && dejerk_error < -_tolerance)
            || (_setpoint > 0 && dejerk_error > _tolerance)) {
        
        _velocity_profile.setpoint(0);
        vpstatus = _velocity_profile.calculate(&vel_seg, &vel_seg_in, time);

        _jerk_out = vel_seg.acceleration;
        segment_out->acceleration = vel_seg.velocity;
        segment_out->velocity = vel_seg.distance;
        segment_out->distance = l_dist + (vel_seg.distance * dt) + 0.5*(vel_seg.velocity)*dt*dt;
        status = Pathfinder::Profile::STATUS_DECEL;
    } else if (fabs(l_vel) < _max_velocity - _tolerance) {
        _velocity_profile.setpoint(_setpoint < 0 ? -_max_velocity : _max_velocity);
        vpstatus = _velocity_profile.calculate(&vel_seg, &vel_seg_in, time);

        _jerk_out = vel_seg.acceleration;
        segment_out->acceleration = vel_seg.velocity;
        segment_out->velocity = vel_seg.distance;
        segment_out->distance = l_dist + (vel_seg.distance * dt) + 0.5*(vel_seg.velocity)*dt*dt;
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