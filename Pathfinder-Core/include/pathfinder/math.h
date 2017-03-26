#pragma once

#include <cmath>

#ifndef M_PI
    #define M_PI 3.14159265
#endif

#define d2r(deg) (deg*M_PI/180.0)
#define r2d(rad) (rad*180.0/M_PI)

double bound_radians(double angle);