#pragma once

#include <PCP/SpacePartitioning/Query/RangePointQuery.h>
#include <PCP/SpacePartitioning/KdTree/Query/KdTreeQuery.h>
#include <PCP/SpacePartitioning/KdTree/Iterator/KdTreeRangePointIterator.h>

namespace pcp {

class KdTreeRangePointQuery : public KdTreeQuery,
                              public RangePointQuery
{
protected:
    friend class KdTreeRangePointIterator;

public:
    KdTreeRangePointQuery();
    KdTreeRangePointQuery(const KdTree* kdtree);
    KdTreeRangePointQuery(const KdTree* kdtree, Scalar radius);
    KdTreeRangePointQuery(const KdTree* kdtree, Scalar radius, const Vector3& point);

public:
    KdTreeRangePointIterator begin();
    KdTreeRangePointIterator end();

protected:
    void initialize(KdTreeRangePointIterator& iterator);
    void advance(KdTreeRangePointIterator& iterator);
};

} // namespace pcp
