#pragma once

#include <PCP/Common/Container/limited_priority_queue.h>

#include <PCP/SpacePartitioning/internal/IndexSquaredDistance.h>

namespace pcp {

class KNearestQuery
{
public:
    KNearestQuery();
    KNearestQuery(int k);

public:
    int k() const;
    void set_k(int k);

public:
    limited_priority_queue<IndexSquaredDistance>& queue();

protected:
    limited_priority_queue<IndexSquaredDistance> m_queue;
};

} // namespace pcp
