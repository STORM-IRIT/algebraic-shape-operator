#pragma once

#include <PCP/Curvature/PointWiseCurvatureData.h>

namespace pcp {

class GlobalCurvatureData
{
public:
    GlobalCurvatureData(int size = 0);

public:
    void resize(int size);
    void push_back(Scalar k1, Scalar k2, Scalar H, const Vector3& dir1, const Vector3& dir2, const Vector3& normal);

public:
    int size() const;

public:
    const PointWiseCurvatureData& operator[](int i) const;
          PointWiseCurvatureData& operator[](int i);

public:
    Scalar k1(int i) const;
    Scalar k2(int i) const;
    const Vector3& dir1(int i) const;
    const Vector3& dir2(int i) const;
    const Vector3& normal(int i) const;

public:
    Scalar& k1(int i);
    Scalar& k2(int i);
    Vector3& dir1(int i);
    Vector3& dir2(int i);
    Vector3& normal(int i);

public:
    std::vector<PointWiseCurvatureData> m_data;
};

} // namespace pcp
