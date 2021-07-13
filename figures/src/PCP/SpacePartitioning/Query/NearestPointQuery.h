#pragma once

#include <PCP/SpacePartitioning/Query/PointQuery.h>
#include <PCP/SpacePartitioning/Query/NearestQuery.h>

namespace pcp {

class NearestPointQuery : public PointQuery,
                          public NearestQuery
{
public:
    NearestPointQuery();
    NearestPointQuery(const Vector3& point);
};

} // namespace pcp
