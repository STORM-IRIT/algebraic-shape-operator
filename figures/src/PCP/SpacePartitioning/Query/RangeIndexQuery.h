#pragma once

#include <PCP/SpacePartitioning/Query/RangeQuery.h>
#include <PCP/SpacePartitioning/Query/IndexQuery.h>

namespace pcp {

class RangeIndexQuery : public IndexQuery,
                        public RangeQuery
{
public:
    RangeIndexQuery();
    RangeIndexQuery(Scalar radius);
    RangeIndexQuery(Scalar radius, int index);
};

} // namespace pcp
