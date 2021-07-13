#pragma once

#include <PCP/SpacePartitioning/Query/RangeIndexQuery.h>
#include <PCP/SpacePartitioning/KdTree/Query/KdTreeQuery.h>
#include <PCP/SpacePartitioning/KdTree/Iterator/KdTreeRangeIndexIterator.h>

namespace pcp {

class KdTreeRangeIndexQuery : public KdTreeQuery,
                              public RangeIndexQuery
{
protected:
    friend class KdTreeRangeIndexIterator;

public:
    KdTreeRangeIndexQuery();
    KdTreeRangeIndexQuery(const KdTree* kdtree);
    KdTreeRangeIndexQuery(const KdTree* kdtree, Scalar radius);
    KdTreeRangeIndexQuery(const KdTree* kdtree, Scalar radius, int index);

public:
    KdTreeRangeIndexIterator begin();
    KdTreeRangeIndexIterator end();

protected:
    void initialize(KdTreeRangeIndexIterator& iterator);
    void advance(KdTreeRangeIndexIterator& iterator);
};

} // namespace pcp
