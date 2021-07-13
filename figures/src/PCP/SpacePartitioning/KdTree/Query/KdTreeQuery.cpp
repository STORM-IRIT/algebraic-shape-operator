#include <PCP/SpacePartitioning/KdTree/Query/KdTreeQuery.h>

namespace pcp {

KdTreeQuery::KdTreeQuery() :
    m_kdtree(nullptr)
{
}

KdTreeQuery::KdTreeQuery(const KdTree* kdtree) :
    m_kdtree(kdtree)
{
}

} // namespace pcp
