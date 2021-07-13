#pragma once

#include <PCP/SpacePartitioning/Query/PointQuery.h>
#include <PCP/SpacePartitioning/Query/KNearestQuery.h>

namespace pcp {

class KNearestPointQuery : public PointQuery,
                           public KNearestQuery
{
public:
    KNearestPointQuery();
    KNearestPointQuery(const Vector3& point);
    KNearestPointQuery(int k);
    KNearestPointQuery(int k, const Vector3& point);
};

} // namespace pcp
