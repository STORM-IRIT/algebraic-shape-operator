#pragma once

#include <PCP/SpacePartitioning/internal/IndexSquaredDistance.h>
#include <PCP/Common/Container/static_stack.h>

#define PCP_KDTREE_MAX_DEPTH 32

namespace pcp {

class KdTree;

class KdTreeQuery
{
public:
    KdTreeQuery();
    KdTreeQuery(const KdTree* kdtree);

protected:
    const KdTree* m_kdtree;
    static_stack<IndexSquaredDistance, 2*PCP_KDTREE_MAX_DEPTH> m_stack;
};

} // namespace pcp
