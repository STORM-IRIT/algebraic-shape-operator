#pragma once

#include <PCP/SpacePartitioning/Query/KNearestPointQuery.h>
#include <PCP/SpacePartitioning/KdTree/Query/KdTreeQuery.h>
#include <PCP/SpacePartitioning/KdTree/Iterator/KdTreeKNearestPointIterator.h>

namespace pcp {

class KdTreeKNearestPointQuery : public KdTreeQuery,
                                 public KNearestPointQuery
{
public:
    KdTreeKNearestPointQuery();
    KdTreeKNearestPointQuery(const KdTree* kdtree, int k);
    KdTreeKNearestPointQuery(const KdTree* kdtree, int k, const Vector3& point);

public:
    KdTreeKNearestPointIterator begin();
    KdTreeKNearestPointIterator end();

public:
    const limited_priority_queue<IndexSquaredDistance>& search();
};

} // namespace pcp
