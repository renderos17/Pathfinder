#include "pathfinder/trajectory.h"
#include <inttypes.h>

void Pathfinder::trajectory(Pathfinder::Spline::Spline *splines, unsigned int spline_count, size_t spline_size, unsigned int samples, Pathfinder::Segment *segments, unsigned int segment_count, Pathfinder::Segment2D *seg2d_ext) {
    unsigned int i, spline_index = 0;
    float spline_distance = splines[0].arc_length(samples);
    float spline_distance_offset = 0;
    for (i = 0; i < segment_count; i++) {
        float pos = segments[i].distance;
        int found = 0;
        while (!found) {
            float pos_rel_spline = pos - spline_distance_offset;
            Pathfinder::Spline::Spline *spline = (Pathfinder::Spline::Spline *)((uint8_t *)splines + spline_index*spline_size);
            if (pos_rel_spline <= spline_distance) {
                float spline_t = pos_rel_spline / spline_distance;
                Pathfinder::Spline::SplineCoord coord;
                spline->calculate(&coord, spline_t);
                seg2d_ext[i].x = coord.x;
                seg2d_ext[i].y = coord.y;
                seg2d_ext[i].angle = coord.angle;
                found = 1;
            } else if (spline_index < spline_count) {
                spline_distance_offset += spline_distance;
                spline_index += 1;
                spline_distance = ((Pathfinder::Spline::Spline *)((uint8_t *)splines + spline_index*spline_size))->arc_length(samples);
            } else {
                Pathfinder::Spline::SplineCoord coord;
                spline->calculate(&coord, 1.0);
                seg2d_ext[i].x = coord.x;
                seg2d_ext[i].y = coord.y;
                seg2d_ext[i].angle = coord.angle;
                found = 1;
            }
        }
    }
}