#include "pathfinder/profile/trapezoidal.h"
#include "pathfinder/math.h"

void Pathfinder::Profile::Trapezoidal::configure(float max_velocity, float acceleration, float tolerance) {
    _max_velocity = max_velocity;
    _acceleration = acceleration;
    _tolerance = tolerance;

    _slconfigured = false;
}

void Pathfinder::Profile::Trapezoidal::configure_shift(ShiftLevel *levels, int level_count) {
    _slvls = levels;
    _slcount = level_count;
    _slconfigured = true;

    set_shift(0);
}

int Pathfinder::Profile::Trapezoidal::shift_level() {
    return _slcurrent;
}

void Pathfinder::Profile::Trapezoidal::set_shift(int level) {
    _slcurrent = level;
    _max_velocity = _slvls[_slcurrent].max_velocity;
    _acceleration = _slvls[_slcurrent].acceleration;
}

uint8_t Pathfinder::Profile::Trapezoidal::calculate(Pathfinder::Segment *segment_out, Pathfinder::Segment *last_segment, float time) {
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
    
    segment_out->time = time;
    float dt = time - l_time;

    if (fabs(l_dist - _setpoint) <= _tolerance) {
        // Drop all to 0 and return
        segment_out->acceleration = 0;
        segment_out->velocity = 0;
        segment_out->distance = l_dist;
        status = Pathfinder::Profile::STATUS_DONE;
        // We don't need to calculate shifting if we've finished the path...
        return status;
    }

    float accel = 0;
    if (l_dist < _setpoint) {
        // Below setpoint, accelerate to setpoint (+ve)
        accel = _acceleration;
    } else {
        // Above setpoint, accelerate to setpoint (-ve)
        accel = -_acceleration;
    }

    float   decel_time = l_vel / accel,
            decel_dist = l_vel * decel_time - 0.5*accel*decel_time*decel_time;
    
    float   decel_error = l_dist + decel_dist - _setpoint;

    float   sixth = 1 / 6.0;

    if (fabs(decel_error) <= _tolerance 
            || (_setpoint < 0 && decel_error < _tolerance) 
            || (_setpoint > 0 && decel_error > _tolerance)) {

        segment_out->acceleration = -accel;
        segment_out->velocity = l_vel - (accel * dt);
        segment_out->distance = l_dist + (l_vel * dt) - (0.5 * accel * dt * dt);
        _distance_integral += l_dist * dt + (0.5 * l_vel * dt*dt) + (sixth * (-accel) * dt*dt*dt);
        status = Pathfinder::Profile::STATUS_DECEL;
    } else if (fabs(l_vel) < _max_velocity) {
        segment_out->acceleration = accel;
        float v = l_vel + (accel * dt);
        segment_out->velocity = (v < -_max_velocity ? -_max_velocity : (v > _max_velocity ? _max_velocity : v));
        segment_out->distance = l_dist + (l_vel * dt) + (0.5 * accel * dt * dt);
        _distance_integral += l_dist * dt + (0.5 * l_vel * dt*dt) + (sixth * accel * dt*dt*dt);
        status = Pathfinder::Profile::STATUS_ACCEL;
    }

    if (status == Pathfinder::Profile::STATUS_LEVEL) {
        // If nothing else, we have levelled out, hold steady velocity
        segment_out->acceleration = 0;
        segment_out->velocity = l_vel;
        segment_out->distance = l_dist + (l_vel * dt);
        _distance_integral += l_dist * dt + (0.5 * l_vel * dt*dt);
    } else if (_slconfigured) {
        float cur_v = fabs(segment_out->velocity);
        if (_slcurrent + 1 < _slcount) {
            ShiftLevel *up = (_slvls + _slcurrent + 1);
            if (cur_v > up->threshold_velocity) {
                set_shift(++_slcurrent);
            }
        }
        
        if (_slcurrent > 0) {
            if (cur_v < _slvls[_slcurrent].threshold_velocity) {
                set_shift(--_slcurrent);
            }
        }
    }
    return status;
}