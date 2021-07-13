#include <PCP/SpacePartitioning/Query/NearestIndexQuery.h>

namespace pcp {

NearestIndexQuery::NearestIndexQuery() :
    IndexQuery(),
    NearestQuery()
{
}

NearestIndexQuery::NearestIndexQuery(int index) :
    IndexQuery(index),
    NearestQuery()
{
}

} // namespace pcp
