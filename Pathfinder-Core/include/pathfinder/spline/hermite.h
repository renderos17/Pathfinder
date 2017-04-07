#pragma once

#include "pathfinder/spline/spline.h"

namespace Pathfinder {
    namespace Spline {
        enum Hermite_Type {
            HermiteCubic, HermiteQuintic
        };
        struct Hermite : Pathfinder::Spline::Spline {
            struct Waypoint {
                double x, y, angle;
            };

            Hermite() {};
            Hermite(Hermite_Type type, Waypoint *start, Waypoint *end) {
                configure(type, start, end);
            }
            void configure(Hermite_Type type, Waypoint *start, Waypoint *end);
            
            void calculate(Pathfinder::Spline::SplineCoord *out, double time);
            double deriv(double time);
            double arc_length(unsigned int samples);

            Waypoint *start_point, *end_point;
            double xoffset, yoffset, aoffset, hyp_distance, tangent0, tangent1;
            double a, b;
            
            // Used to cache Arc Length calculation, as it's quite CPU intensive
            double last_arc_calc;
            int last_arc_calc_samples = 0;
            Hermite_Type fit_type;
        };

        int hermite(Hermite_Type type, Hermite::Waypoint *waypoints, unsigned int waypoint_count, Hermite *splines_out);
    }
}