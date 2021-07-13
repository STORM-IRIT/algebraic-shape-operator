#pragma once

#include <PCP/Common/Scalar.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace pcp {

using VectorX  = Eigen::Matrix<Scalar,Eigen::Dynamic,1>;
using Vector2  = Eigen::Matrix<Scalar,2,1>;
using Vector3  = Eigen::Matrix<Scalar,3,1>;
using Vector4  = Eigen::Matrix<Scalar,4,1>;
using Vector3i = Eigen::Matrix<int,3,1>;

using Vector2Array  = std::vector<Vector2>;
using Vector3Array  = std::vector<Vector3>;
using Vector4Array  = std::vector<Vector4>;
using Vector3iArray = std::vector<Vector3i>;

using Matrix   = Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic>;
using Matrix2  = Eigen::Matrix<Scalar,2,2>;
using Matrix3  = Eigen::Matrix<Scalar,3,3>;
using Matrix32 = Eigen::Matrix<Scalar,3,2>;

using Aabb = Eigen::AlignedBox<Scalar,3>;

} // namespace pcp
