#include <PCP/SpacePartitioning/Query/RangePointQuery.h>

namespace pcp {

RangePointQuery::RangePointQuery() :
    PointQuery(),
    RangeQuery()
{
}

RangePointQuery::RangePointQuery(Scalar radius) :
    PointQuery(),
    RangeQuery(radius)
{
}

RangePointQuery::RangePointQuery(Scalar radius, const Vector3& point) :
    PointQuery(point),
    RangeQuery(radius)
{
}

} // namespace pcp
