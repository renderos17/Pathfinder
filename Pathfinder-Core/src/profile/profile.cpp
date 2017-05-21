#include "pathfinder/profile/profile.h"

uint8_t Pathfinder::Profile::Profile::calculate(Pathfinder::Segment *out, Pathfinder::Segment *last, float time) {
    Pathfinder::Segment temporary = { last->time, last->distance, last->velocity, last->acceleration };
    float dt = time - last->time;
    int slice_count = (int)(dt / _timescale);

    int i;
    uint8_t result;
    if (slice_count < 1) {
        // The time difference provided is smaller than the target timescale, 
        // use the smaller of the two.
        result = calculate_single(&temporary, &temporary, time);
    } else {
        for (i = 1; i <= slice_count; i++) {
            float time_slice = temporary.time + _timescale;
            result = calculate_single(&temporary, &temporary, time_slice);
            if (result == Pathfinder::Profile::STATUS_DONE) break;
        }
    }
    out->time = temporary.time;
    out->distance = temporary.distance;
    out->velocity = temporary.velocity; 
    out->acceleration = temporary.acceleration;
    return result;
}