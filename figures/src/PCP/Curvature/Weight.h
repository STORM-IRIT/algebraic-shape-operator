#include <PCP/Geometry/Defines.h>

namespace pcp {

inline Scalar weight(const Vector3& p, Scalar r)
{
    const Scalar w = p.squaredNorm() / (r*r) - 1;
    return w*w;
}

inline Scalar weight(const Vector3& p, const Vector3& q, Scalar r)
{
    return weight(p - q, r);
}

} // namespace pcp
