#include <PCP/Curvature/GlobalCurvatureData.h>

namespace pcp {

GlobalCurvatureData::GlobalCurvatureData(int size) : m_data(size)
{
}

void GlobalCurvatureData::resize(int size)
{
    m_data.resize(size);
}

void GlobalCurvatureData::push_back(Scalar k1, Scalar k2, Scalar H, const Vector3& dir1, const Vector3& dir2, const Vector3& normal)
{
    m_data.push_back(PointWiseCurvatureData(k1, k2, H, normal, dir1, dir2));
}

int GlobalCurvatureData::size() const
{
    return m_data.size();
}

const PointWiseCurvatureData& GlobalCurvatureData::operator[](int i) const
{
    return m_data[i];
}

PointWiseCurvatureData& GlobalCurvatureData::operator[](int i)
{
    return m_data[i];
}

Scalar GlobalCurvatureData::k1(int i) const
{
    return m_data[i].m_k1;
}

Scalar GlobalCurvatureData::k2(int i) const
{
    return m_data[i].m_k2;
}

const Vector3& GlobalCurvatureData::dir1(int i) const
{
    return m_data[i].m_dir1;
}

const Vector3& GlobalCurvatureData::dir2(int i) const
{
    return m_data[i].m_dir2;
}

const Vector3& GlobalCurvatureData::normal(int i) const
{
    return m_data[i].m_N;
}

Scalar& GlobalCurvatureData::k1(int i)
{
    return m_data[i].m_k1;
}

Scalar& GlobalCurvatureData::k2(int i)
{
    return m_data[i].m_k2;
}

Vector3& GlobalCurvatureData::dir1(int i)
{
    return m_data[i].m_dir1;
}

Vector3& GlobalCurvatureData::dir2(int i)
{
    return m_data[i].m_dir2;
}

Vector3& GlobalCurvatureData::normal(int i)
{
    return m_data[i].m_N;
}

} // namespace pcp
























