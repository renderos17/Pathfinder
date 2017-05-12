#pragma once

#include <cmath>

#ifndef M_PI
    #define M_PI 3.14159265
#endif

#ifndef MIN
    #define MIN(a,b) (((a)<(b))?(a):(b))
    #define MAX(a,b) (((a)>(b))?(a):(b))
#endif

#ifndef SAME_SIGN
    #define SAME_SIGN(x,y) ((x<0) == (y<0))
#endif

#define d2r(deg) (deg*M_PI/180.0)
#define r2d(rad) (rad*180.0/M_PI)

double bound_radians(double angle);