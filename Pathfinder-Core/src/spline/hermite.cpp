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

    fit_type = type;

    last_arc_calc_samples = 0;
}

void Pathfinder::Spline::Hermite::calculate(Pathfinder::Spline::SplineCoord *out, double t) {
    out->time = t;
    double x = hyp_distance * t;
    double y = 0;
    if (fit_type == HermiteCubic) {
        y = ((tangent0 + tangent1) * hyp_distance * t*t*t)
            + (-(2 * tangent0 + tangent1) * hyp_distance * t*t)
            + tangent0 * hyp_distance * t;
    } else if (fit_type == HermiteQuintic) {
        y = (-(3 * (tangent0+tangent1))) * hyp_distance * t*t*t*t*t
            + (8 * tangent0 + 7 * tangent1) * hyp_distance * t*t*t*t
            + (-(6 * tangent0 + 4 * tangent1)) * hyp_distance * t*t*t
            + (tangent0) * hyp_distance * t;
    }

    // Translate back to global x/y axis
    out->x = x*cos(aoffset) - y*sin(aoffset) + xoffset;
    out->y = x*sin(aoffset) + y*cos(aoffset) + yoffset;
    out->angle = bound_radians(atan(deriv(t)) + aoffset);
}

double Pathfinder::Spline::Hermite::deriv(double t) {
    double x = hyp_distance * t;
    if (fit_type == HermiteCubic) {
        return (3 * (tangent0 + tangent1) * t*t) 
                + (2*(-(2 * tangent0 + tangent1) * t)) 
                + tangent0;
    } else if (fit_type == HermiteQuintic) {
        return 5 * (-(3*(tangent0 + tangent1))) * t*t*t*t
                + 4 * (8 * tangent0 + 7 * tangent1) * t*t*t
                + 3 * (-(6 * tangent0 + 4 * tangent1)) * t*t
                + tangent0;
    }
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
    for (i = 1; i < waypoint_count; i++) {
        if (waypoints[i].angle == Pathfinder::Spline::Hermite::ANGLE_AUTO) {
            double dx = waypoints[i].x - waypoints[i-1].x;
            double dy = waypoints[i].y - waypoints[i-1].y;
            waypoints[i].angle = atan2(dy, dx);
        }
    }
    
    for (i = 0; i < waypoint_count - 1; i++) {
        splines_out[i].configure(type, &waypoints[i], &waypoints[i+1]);
    }
    return waypoint_count - 1;
}