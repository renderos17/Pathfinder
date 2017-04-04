#include "pathfinder/segments.h"
#include "pathfinder/spline/spline.h"

#include <inttypes.h>

namespace Pathfinder {
    namespace Trajectory {
        struct Coupled {
            Coupled() { }
            template<class SplineClass>
            void configure_path(SplineClass *splines, uint8_t spline_count, uint32_t spline_samples) {

            }

            void configure_profile(Pathfinder::Profile::Profile *profile) {
                
            }
        }
    }
}