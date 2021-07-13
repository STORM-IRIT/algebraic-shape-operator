#include <PCP/SpacePartitioning/Query/NearestPointQuery.h>

namespace pcp {

NearestPointQuery::NearestPointQuery() :
    PointQuery(),
    NearestQuery()
{
}

NearestPointQuery::NearestPointQuery(const Vector3& point) :
    PointQuery(point),
    NearestQuery()
{
}

} // namespace pcp
