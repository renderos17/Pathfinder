#pragma once

namespace Pathfinder {
    namespace Spline {
        struct SplineCoord {
            double time, x, y, angle;
        };

        struct Spline {
            virtual void calculate(Pathfinder::Spline::SplineCoord *out, double time) = 0;
            virtual double deriv(double time) = 0;
            virtual double arc_length(unsigned int samples) = 0;
        };

        template<class T>
        double distance(T *splines, unsigned int spline_count, unsigned int samples) {
            int i;
            double dist = 0;
            for (i = 0; i < spline_count; i++) {
                dist += splines[i].arc_length(samples);
            }
            return dist;
        }
    }
}