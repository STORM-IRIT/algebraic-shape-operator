#include <PCP/Geometry/Point.h>
#include <PCP/Geometry/Geometry.h>

namespace pcp {

ConstPoint::ConstPoint() :
    m_geometry(nullptr),
    m_index(-1)
{
}

ConstPoint::ConstPoint(const Geometry* geometry, int index) :
    m_geometry(geometry),
    m_index(index)
{
}

int ConstPoint::index() const
{
    return m_index;
}

int& ConstPoint::index()
{
    return m_index;
}

const Vector3& ConstPoint::pos() const
{
    return m_geometry->point(m_index);
}

const Vector3& ConstPoint::point() const
{
    return m_geometry->point(m_index);
}

const Vector3& ConstPoint::normal() const
{
    return m_geometry->normal(m_index);
}

const Vector4& ConstPoint::color() const
{
    return m_geometry->color(m_index);
}

////////////////////////////////////////////////////////////////////////////////

Point::Point() :
    ConstPoint()
{
}

Point::Point(Geometry* geometry, int index) :
    ConstPoint(geometry, index)
{
}

Vector3& Point::pos()
{
    return geometry()->point(m_index);
}

Vector3& Point::point()
{
    return geometry()->point(m_index);
}

Vector3& Point::normal()
{
    return geometry()->normal(m_index);
}

Vector4& Point::color()
{
    return geometry()->color(m_index);
}

const Vector3& Point::pos() const
{
    return m_geometry->point(m_index);
}

const Vector3& Point::point() const
{
    return m_geometry->point(m_index);
}

const Vector3& Point::normal() const
{
    return m_geometry->normal(m_index);
}

const Vector4& Point::color() const
{
    return m_geometry->color(m_index);
}

Geometry* Point::geometry()
{
    return const_cast<Geometry*>(m_geometry);
}

} // namespace pcp
