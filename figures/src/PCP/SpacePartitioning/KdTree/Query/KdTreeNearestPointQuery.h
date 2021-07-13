#pragma once

#include <PCP/SpacePartitioning/Query/NearestPointQuery.h>
#include <PCP/SpacePartitioning/KdTree/Query/KdTreeQuery.h>
#include <PCP/SpacePartitioning/KdTree/Iterator/KdTreeNearestPointIterator.h>

namespace pcp {

class KdTreeNearestPointQuery : public KdTreeQuery,
                                public NearestPointQuery
{
public:
    KdTreeNearestPointQuery();
    KdTreeNearestPointQuery(const KdTree* kdtree);
    KdTreeNearestPointQuery(const KdTree* kdtree, const Vector3& point);

public:
    KdTreeNearestPointIterator begin();
    KdTreeNearestPointIterator end();

protected:
    void search();
};

} // namespace pcp
