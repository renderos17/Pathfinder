#include "pathfinder/math.h"

double bound_radians(double angle) {
    double newAngle = fmod(angle, 2*M_PI);
    if (newAngle < 0) newAngle = 2*M_PI + newAngle;
    return newAngle;
}