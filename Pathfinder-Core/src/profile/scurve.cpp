#include "pathfinder/profile/scurve.h"
#include "pathfinder/math.h"

#include <mutex>

static float f1_buffer[8192];
static std::mutex mtx;

int Pathfinder::Profile::scurve(Pathfinder::Segment *out, float timescale, float distance, float max_velocity, float max_acceleration, float jerk) {
    float ma2 = max_acceleration * max_acceleration;
    float mj2 = jerk * jerk;

    float checked_max_velocity = MIN(
        max_velocity,
        (-ma2 + sqrt(ma2 * ma2 + 4 * (mj2 * max_acceleration * distance))) / (2 * jerk)
    );

    int filter_1 = (int)ceil(checked_max_velocity / max_acceleration / timescale);
    int filter_2 = (int)ceil(max_acceleration / jerk / timescale);

    float impulse = distance / checked_max_velocity / timescale;
    int len = (int)ceil(filter_1 + filter_2 + impulse);

    // Mutex needed for f1_buffer
    std::lock_guard<std::mutex> guard(mtx);
    float f1_last = 0, f2 = 0, last_vel = 0, last_dist = 0, last_acc = 0;
    int i;
    for (i = 0; i < len; i++) {
        float input = MIN(impulse, 1);
        if (input < 1) {
            input -= 1;
            impulse = 0;
        } else {
            impulse -= input;
        }

        f1_buffer[i] = MAX(0.0, MIN(filter_1, f1_last + input));
        f2 = 0;
        int j;
        for (j = 0; j < filter_2; j++) {
            if (i - j < 0) break;
            f2 += f1_buffer[i - j];
        }
        f2 = f2 / filter_1;

        out[i].timescale = timescale;
        out[i].velocity = f2 / filter_2 * checked_max_velocity;
        out[i].distance = (last_vel + out[i].velocity) / 2.0 * timescale + last_dist; // s = (0.5*u*v)t + s0
        out[i].acceleration = (out[i].velocity - last_vel) / timescale;

        last_vel = out[i].velocity;
        last_dist = out[i].distance;
        last_acc = out[i].acceleration;

        f1_last = f1_buffer[i];
    }
    return len;
}