#pragma once

namespace pcp {

class KdTreeNearestPointIterator
{
public:
    KdTreeNearestPointIterator();
    KdTreeNearestPointIterator(int index);

public:
    bool operator !=(const KdTreeNearestPointIterator& other) const;
    void operator ++();
    int  operator * () const;

protected:
    int m_index;
};

} // namespace pcp
