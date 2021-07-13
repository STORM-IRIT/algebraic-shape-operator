#pragma once

#include <PCP/SpacePartitioning/Query/NearestIndexQuery.h>
#include <PCP/SpacePartitioning/KdTree/Query/KdTreeQuery.h>
#include <PCP/SpacePartitioning/KdTree/Iterator/KdTreeNearestIndexIterator.h>

namespace pcp {

class KdTreeNearestIndexQuery : public KdTreeQuery,
                                public NearestIndexQuery
{
public:
    KdTreeNearestIndexQuery();
    KdTreeNearestIndexQuery(const KdTree* kdtree);
    KdTreeNearestIndexQuery(const KdTree* kdtree, int index);

public:
    KdTreeNearestIndexIterator begin();
    KdTreeNearestIndexIterator end();

public:
    const NearestIndexQuery& search();
};

} // namespace pcp
