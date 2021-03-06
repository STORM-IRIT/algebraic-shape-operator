#pragma once

namespace pcp {

class KdTreeNearestIndexIterator
{
public:
    KdTreeNearestIndexIterator();
    KdTreeNearestIndexIterator(int index);

public:
    bool operator !=(const KdTreeNearestIndexIterator& other) const;
    void operator ++();
    int  operator * () const;

protected:
    int m_index;
};

} // namespace pcp
