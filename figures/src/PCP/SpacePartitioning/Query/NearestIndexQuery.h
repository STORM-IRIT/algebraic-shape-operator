#pragma once

#include <PCP/SpacePartitioning/Query/IndexQuery.h>
#include <PCP/SpacePartitioning/Query/NearestQuery.h>

namespace pcp {

class NearestIndexQuery : public IndexQuery,
                          public NearestQuery
{
public:
    NearestIndexQuery();
    NearestIndexQuery(int index);
};

} // namespace pcp
