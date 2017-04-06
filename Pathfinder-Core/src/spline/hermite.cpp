#include "pathfinder/spline/hermite.h"
#include "pathfinder/math.h"
#include <string>

void Pathfinder::Spline::Hermite::configure(Pathfinder::Spline::Hermite_Type type, Pathfinder::Spline::Hermite::Waypoint *start, Pathfinder::Spline::Hermite::Waypoint *end) {
    start_point = start;
    end_point = end;

    xoffset = start->x;
    yoffset = start->y;
    double dy = end->y - start->y;
    double dx = end->x - start->x;
    aoffset = atan2(dy, dx);
    hyp_distance = sqrt(dx*dx + dy*dy);

    tangent0 = tan(start->angle - aoffset);
    tangent1 = tan(end->angle - aoffset);

    a = tangent0 * hyp_distance;
    b = tangent1 * hyp_distance;

    last_arc_calc_samples = 0;
}

void Pathfinder::Spline::Hermite::calculate(Pathfinder::Spline::SplineCoord *out, double t) {
    out->time = t;
    double x = hyp_distance * t;
    double y = 0;
    // TODO Quintic Support
    if (fit_type == HermiteCubic) {
        // TODO: Clean this up
        y = ((tangent0 + tangent1) / (hyp_distance * hyp_distance))*x*x*x
            + (-(2 * tangent0 + tangent1) / hyp_distance)*x*x
            + tangent0 * x;
    }

    // Translate back to global x/y axis
    out->x = x*cos(aoffset) - y*sin(aoffset) + xoffset;
    out->y = x*sin(aoffset) + y*cos(aoffset) + yoffset;
    out->angle = bound_radians(atan(deriv(t)) + aoffset);
}

double Pathfinder::Spline::Hermite::deriv(double t) {
    double x = hyp_distance * t;
    // TODO: Clean this up
    return (3 * ((tangent0 + tangent1) / (hyp_distance * hyp_distance))*x*x) 
            + (2*(-(2 * tangent0 + tangent1) / hyp_distance)*x) 
            + tangent0;
}

double Pathfinder::Spline::Hermite::arc_length(unsigned int samples) {
    if (last_arc_calc_samples != samples) {
        double t = 0, dt = (1.0f/samples);

        double dydt = deriv(t);
        double integrand = 0;
        double arc_length = 0;
        double last_integrand = sqrt(1 + dydt*dydt) * dt;

        for (t = 0; t <= 1; t += dt) {
            dydt = deriv(t);
            integrand = sqrt(1 + dydt*dydt) * dt;
            arc_length += (integrand + last_integrand) / 2;
            last_integrand = integrand;
        }
        double al = hyp_distance * arc_length;
        last_arc_calc_samples = samples;
        last_arc_calc = al;
        return al;
    }
    return last_arc_calc;
}

int Pathfinder::Spline::hermite(Pathfinder::Spline::Hermite_Type type, Pathfinder::Spline::Hermite::Waypoint *waypoints, unsigned int waypoint_count, Pathfinder::Spline::Hermite *splines_out) {
    int i;
    for (i = 0; i < waypoint_count - 1; i++) {
        splines_out[i].configure(type, &waypoints[i], &waypoints[i+1]);
    }
    return waypoint_count - 1;
}