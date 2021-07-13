#pragma once

#include <PCP/SpacePartitioning/Query/IndexQuery.h>
#include <PCP/SpacePartitioning/Query/KNearestQuery.h>

namespace pcp {

class KNearestIndexQuery : public IndexQuery,
                           public KNearestQuery
{
public:
    KNearestIndexQuery();
    KNearestIndexQuery(int k);
    KNearestIndexQuery(int k, int index);
};

} // namespace pcp
