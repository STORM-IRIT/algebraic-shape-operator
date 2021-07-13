#pragma once

#include <PCP/Geometry/Defines.h>

namespace pcp {

class PointWiseCurvatureData
{
public:
    PointWiseCurvatureData() :
        m_k1(666),
        m_k2(666),
        m_H(666),
        m_N(Vector3::Constant(666)),
        m_dir1(Vector3::Constant(666)),
        m_dir2(Vector3::Constant(666))
    {
    }

    PointWiseCurvatureData(Scalar k1,
                           Scalar k2,
                           Scalar H,
                           const Vector3& normal,
                           const Vector3& dir1,
                           const Vector3& dir2) :
        m_k1(k1),
        m_k2(k2),
        m_H(H),
        m_N(normal),
        m_dir1(dir1),
        m_dir2(dir2)
    {}

public:
    inline Scalar k1() const {return m_k1;}
    inline Scalar k2() const {return m_k2;}
    inline Scalar H() const {return m_H;}
    inline const Vector3& normal() const {return m_N;}
    inline const Vector3& dir1() const {return m_dir1;}
    inline const Vector3& dir2() const {return m_dir2;}

    inline Scalar& k1() {return m_k1;}
    inline Scalar& k2() {return m_k2;}
    inline Scalar& H() {return m_H;}
    inline const Vector3& normal() {return m_N;}
    inline const Vector3& dir1() {return m_dir1;}
    inline const Vector3& dir2() {return m_dir2;}

    inline Scalar K() const {return k1() * k2();}

public:
    Scalar  m_k1;
    Scalar  m_k2;
    Scalar  m_H;
    Vector3 m_N;
    Vector3 m_dir1;
    Vector3 m_dir2;
};

} // namespace pcp
