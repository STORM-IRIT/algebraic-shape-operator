#pragma once

#include <PCP/SpacePartitioning/Query/KNearestQuery.h>

namespace pcp {

class KdTreeKNearestPointIterator
{
public:
    KdTreeKNearestPointIterator();
    KdTreeKNearestPointIterator(limited_priority_queue<IndexSquaredDistance>::iterator iterator);

public:
    bool operator !=(const KdTreeKNearestPointIterator& other) const;
    void operator ++();
    int  operator * () const;

protected:
    limited_priority_queue<IndexSquaredDistance>::iterator m_iterator;
};

} // namespace pcp
