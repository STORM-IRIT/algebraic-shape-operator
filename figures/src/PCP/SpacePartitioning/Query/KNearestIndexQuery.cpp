#include <PCP/SpacePartitioning/Query/KNearestIndexQuery.h>

namespace pcp {

KNearestIndexQuery::KNearestIndexQuery() :
    IndexQuery(),
    KNearestQuery()
{
}

KNearestIndexQuery::KNearestIndexQuery(int k) :
    IndexQuery(),
    KNearestQuery(k)
{
}

KNearestIndexQuery::KNearestIndexQuery(int k, int index) :
    IndexQuery(index),
    KNearestQuery(k)
{
}

} // namespace pcp
