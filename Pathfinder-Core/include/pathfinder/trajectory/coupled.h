#include "pathfinder/segments.h"
#include "pathfinder/spline/spline.h"
#include "pathfinder/profile/profile.h"

#include <cinttypes>
#include <cstddef>

namespace Pathfinder {
    namespace Trajectory {
        struct CoupledSegment {
            Pathfinder::Segment center, left, right;
            Pathfinder::Segment2D center_2, left_2, right_2;
        };

        struct Coupled {
            Coupled(float wheelbase_width) { _wheelbase = wheelbase_width / 2; }
            template<class SplineClass>
            void configure_path(SplineClass *splines, uint8_t spline_count, uint32_t spline_samples) {
                _splines = splines;
                _spline_size = sizeof(SplineClass);
                _spline_count = spline_count;
                _spline_samples = spline_samples;

                // Pregen spline arc lengths so we're not wasting time in the calculate method
                // Refer to Pathfinder::Spline::Spline for details.
                _total_distance = Pathfinder::Spline::distance(splines, spline_count, spline_samples);
            }

            void configure_profile(Pathfinder::Profile::Profile *profile) {
                _profile = profile;
            }

            int calculate(CoupledSegment *segments_out, CoupledSegment *last_segment, float time);

            Pathfinder::Spline::Spline *_splines;
            size_t _spline_size;
            uint8_t _spline_count;
            uint32_t _spline_samples;

            Pathfinder::Profile::Profile *_profile;

            float _wheelbase, _total_distance;
        };
    }
}