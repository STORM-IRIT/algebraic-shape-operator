#include <PCP/SpacePartitioning/Query/KNearestPointQuery.h>

namespace pcp {

KNearestPointQuery::KNearestPointQuery() :
    PointQuery(),
    KNearestQuery()
{
}

KNearestPointQuery::KNearestPointQuery(const Vector3& point) :
    PointQuery(point),
    KNearestQuery()
{
}

KNearestPointQuery::KNearestPointQuery(int k) :
    PointQuery(),
    KNearestQuery(k)
{
}

KNearestPointQuery::KNearestPointQuery(int k, const Vector3& point) :
    PointQuery(point),
    KNearestQuery(k)
{
}

} // namespace pcp
