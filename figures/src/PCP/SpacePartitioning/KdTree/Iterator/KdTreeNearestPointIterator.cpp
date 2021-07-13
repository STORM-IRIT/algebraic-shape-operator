#include <PCP/SpacePartitioning/KdTree/Iterator/KdTreeNearestPointIterator.h>

namespace pcp {

KdTreeNearestPointIterator::KdTreeNearestPointIterator() :
    m_index(-1)
{
}

KdTreeNearestPointIterator::KdTreeNearestPointIterator(int index) :
    m_index(index)
{
}

bool KdTreeNearestPointIterator::operator !=(const KdTreeNearestPointIterator& other) const
{
    return m_index != other.m_index;
}

void KdTreeNearestPointIterator::operator ++()
{
    ++m_index;
}

int KdTreeNearestPointIterator::operator * () const
{
    return m_index;
}

} // namespace pcp
