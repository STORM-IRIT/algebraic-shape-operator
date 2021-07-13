#pragma once

#include <PCP/Geometry/Point.h>

namespace pcp {

class ConstPointIterator
{
public:
    ConstPointIterator();
    ConstPointIterator(const Geometry* geometry, int index);

public:
    bool operator !=(const ConstPointIterator& other) const;
    void operator ++();
    ConstPoint& operator *();

protected:
    ConstPoint m_point;
};

////////////////////////////////////////////////////////////////////////////////

class PointIterator
{
public:
    PointIterator();
    PointIterator(Geometry* geometry, int index);

public:
    bool operator !=(const PointIterator& other) const;
    void operator ++();
    Point& operator *();

protected:
    Point m_point;
};

} // namespace pcp
