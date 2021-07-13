#include <PCP/Geometry/PointIterator.h>

namespace pcp {

ConstPointIterator::ConstPointIterator() :
    m_point()
{
}

ConstPointIterator::ConstPointIterator(const Geometry* geometry, int index) :
    m_point(geometry, index)
{
}

bool ConstPointIterator::operator !=(const ConstPointIterator& other) const
{
    return m_point.index() != other.m_point.index();
}

void ConstPointIterator::operator ++()
{
    ++m_point.index();
}

ConstPoint& ConstPointIterator::operator *()
{
    return m_point;
}

////////////////////////////////////////////////////////////////////////////////

PointIterator::PointIterator() :
    m_point()
{
}

PointIterator::PointIterator(Geometry* geometry, int index) :
    m_point(geometry, index)
{
}

bool PointIterator::operator !=(const PointIterator& other) const
{
    return m_point.index() != other.m_point.index();
}

void PointIterator::operator ++()
{
    ++m_point.index();
}

Point& PointIterator::operator *()
{
    return m_point;
}

} // namespace pcp
