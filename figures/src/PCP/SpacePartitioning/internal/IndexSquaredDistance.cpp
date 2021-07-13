#include <PCP/SpacePartitioning/internal/IndexSquaredDistance.h>

namespace pcp {

bool IndexSquaredDistance::operator < (const IndexSquaredDistance& other) const
{
    return squared_distance < other.squared_distance;
}

} // namespace pcp
