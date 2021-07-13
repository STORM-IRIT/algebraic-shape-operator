#pragma once

#include <PCP/Geometry/Defines.h>

namespace pcp {

class Geometry;

class ConstPoint
{
public:
    enum{ Dim        = 3};          // patate
    using Scalar     = pcp::Scalar; // patate
    using VectorType = Vector3;     // patate
    using MatrixType = Matrix3;     // patate

public:
    ConstPoint();
    ConstPoint(const Geometry* geometry, int index);

public:
    int  index() const;
    int& index();

public:
    const Vector3& pos() const;
    const Vector3& point() const;
    const Vector3& normal() const;
    const Vector4& color() const;

protected:
    const Geometry* m_geometry;
    int m_index;
};

////////////////////////////////////////////////////////////////////////////////

class Point : public ConstPoint
{
public:
    Point();
    Point(Geometry* geometry, int index);

public:
    Vector3& pos();
    Vector3& point();
    Vector3& normal();
    Vector4& color();

public:
    const Vector3& pos() const;
    const Vector3& point() const;
    const Vector3& normal() const;
    const Vector4& color() const;

protected:
    Geometry* geometry();
};

} // namespace pcp
