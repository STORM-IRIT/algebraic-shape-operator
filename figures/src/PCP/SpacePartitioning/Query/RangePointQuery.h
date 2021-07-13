#pragma once

#include <PCP/SpacePartitioning/Query/RangeQuery.h>
#include <PCP/SpacePartitioning/Query/PointQuery.h>

namespace pcp {

class RangePointQuery : public PointQuery,
                        public RangeQuery
{
public:
    RangePointQuery();
    RangePointQuery(Scalar radius);
    RangePointQuery(Scalar radius, const Vector3& point);
};

} // namespace pcp
