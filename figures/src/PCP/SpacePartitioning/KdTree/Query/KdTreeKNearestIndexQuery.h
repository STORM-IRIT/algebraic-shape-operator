#pragma once

#include <PCP/SpacePartitioning/Query/KNearestIndexQuery.h>
#include <PCP/SpacePartitioning/KdTree/Query/KdTreeQuery.h>
#include <PCP/SpacePartitioning/KdTree/Iterator/KdTreeKNearestIndexIterator.h>

namespace pcp {

class KdTreeKNearestIndexQuery : public KdTreeQuery,
                                 public KNearestIndexQuery
{
public:
    KdTreeKNearestIndexQuery();
    KdTreeKNearestIndexQuery(const KdTree* kdtree, int k);
    KdTreeKNearestIndexQuery(const KdTree* kdtree, int k, int index);

public:
    KdTreeKNearestIndexIterator begin();
    KdTreeKNearestIndexIterator end();

public:
     const limited_priority_queue<IndexSquaredDistance>& search();
};

} // namespace pcp
