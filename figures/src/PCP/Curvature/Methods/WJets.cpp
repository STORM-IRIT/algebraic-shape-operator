#include <PCP/Curvature/Curvature.h>
#include <PCP/Geometry/Geometry.h>

#include <PCP/SpacePartitioning/KdTree.h>

#include <Eigen/Eigenvalues>

namespace pcp {

// see https://perso.liris.cnrs.fr/julie.digne/articles/compute_wavejets.m
PointWiseEstimationData CurvatureComputer_WJets::compute(const Geometry& points, int i0, Scalar s) const
{
    constexpr auto order = 2;
    constexpr auto I     = std::complex<Scalar>(0,1); // complex number 0+i

    // % compute PCA normal
    Matrix3 C      = Matrix3::Zero();
    Vector3 m      = Vector3::Zero();
    int     nneigh = 0;

    for(int i : points.kdtree().range_neighbors(i0, s))
    {
        m      += points[i];
        C      += points[i] * points[i].transpose();
        nneigh += 1;
    }

    if(nneigh < NEI_COUNT_MIN) return PointWiseEstimationData::Invalid();

    m /= nneigh;
    C = C/nneigh - m * m.transpose();

    // eigenvalues are positive and ordered
    // eigenvectors are normalized
    Eigen::SelfAdjointEigenSolver<Matrix3> solver(C);
//    const Vector3 normal = solver.eigenvectors().col(0);
    const Vector3 t2 = solver.eigenvectors().col(1);
    const Vector3 t1 = solver.eigenvectors().col(2);
    const Vector3 normal = t1.cross(t2);

    Matrix3 P;
    P.col(0) = t1;
    P.col(1) = t2;
    P.col(2) = normal;

    // % compute Phi
    constexpr int ncolM = order*order/2 + 3*order/2+1; // = 6
    PCP_ASSERT(ncolM == 6);

    Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, Eigen::Dynamic> M(nneigh,ncolM);
    Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, 1>              b(nneigh);

    M.setZero();
    b.setZero();

    int nei_count = 0;

    int j=0;
    for(int i : points.kdtree().range_neighbors(i0, s))
    {
        // % express neighbors in local coord system
        const Vector3 locneighbors = P.transpose() * (points[i] - points[i0]);

        // % switch to polar coordinates
        const Scalar r     = locneighbors.head<2>().norm();
        const Scalar theta = std::atan2(locneighbors.y(), locneighbors.x());
              Scalar z     = locneighbors.z();

        const Scalar normalized_r = r / s;
        z = z / s;

        const Scalar w = std::exp(-normalized_r * normalized_r/18.0);

        int idx = 0;
        for(int k=0; k<=order; ++k)
        {
            const Scalar rk = std::pow(normalized_r, k);
            for(int n = -k; n <= k; n += 2)
            {
                M(j,idx) = rk * std::exp(I * Scalar(n) * theta) * w;
                ++idx;
            }
        }
        b[j] = z * w;
        ++j;
        ++nei_count;
    }
    if(nei_count < NEI_COUNT_MIN) return PointWiseEstimationData::Invalid();

    Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, 1> Phi = M.colPivHouseholderQr().solve(b);

    // % correct normal
    Vector3 u = - t1 * std::imag(Phi(1) - Phi(2)) + t2 * std::real(Phi(2) + Phi(1)); // % axis
    u.normalize();
    Scalar gamma = - std::atan( 2 * std::abs(Phi(2))); // % angle

    Vector3 nc = std::cos(gamma) * normal + (1 - std::cos(gamma)) * (normal.dot(u)) * u + std::sin(gamma) * u.cross(normal);
    nc.normalize();
    Vector3 t1c = t1 - (t1.transpose() * nc) * nc;
    t1c.normalize();
    Vector3 t2c = nc.cross(t1c);
    t2c.normalize();

    // % redo everything (instead of correcting the coefficients - quick and dirty method, to be improved later)
    Matrix3 Pc;
    Pc.col(0) = t1c;
    Pc.col(1) = t2c;
    Pc.col(2) = nc;

    M.setZero();
    b.setZero();

    nei_count = 0;

    j=0;
    for(int i : points.kdtree().range_neighbors(i0, s))
    {
        // % express neighbors in local coord system
        const Vector3 locneighbors = Pc.transpose() * (points[i] - points[i0]);   // Pc not P !

        // switch to polar coordinates
        const Scalar r     = locneighbors.head<2>().norm();
        const Scalar theta = std::atan2(locneighbors.y(), locneighbors.x());
              Scalar z     = locneighbors.z();

        const Scalar normalized_r = r / s;
        z = z / s;

        const Scalar w  = std::exp(-normalized_r*normalized_r/18.0);

        int idx = 0;
        for(int k=0; k<=order; ++k)
        {
            const Scalar rk = std::pow(normalized_r, k);
            for(int n = -k; n <= k; n += 2)
            {
                M(j,idx) = rk * std::exp(I * Scalar(n) * theta) * w;
                ++idx;
            }
        }
        b[j] = z * w;
        ++j;
        ++nei_count;
    }
    if(nei_count < NEI_COUNT_MIN) return PointWiseEstimationData::Invalid();

    Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, 1> Phicorr = M.colPivHouseholderQr().solve(b);

    // % compute real(a0) and |an| n>=1
//    idx = 1;
//    an=zeros(order+1,1);
//    for k=0:order
//        for n = -k:2:k
//          if n>=0
//           an(n+1) = an(n+1) + Phicorr(idx)/(k+2);
//          end
//          idx=idx+1;
//        end
//    end
//    an(1)=real(an(1));%imaginary part is 0
//    an(2:end)=abs(an(2:end));


//    const std::complex<Scalar> phi_0_0  = Phicorr(0);
//    const std::complex<Scalar> phi_1_m1 = Phicorr(1);
//    const std::complex<Scalar> phi_1_p1 = Phicorr(2);
    const std::complex<Scalar> phi_2_m2 = Phicorr(3);
    const std::complex<Scalar> phi_2_0  = Phicorr(4);
    const std::complex<Scalar> phi_2_p2 = Phicorr(5);

    const Vector3 N = nc; // corrected normal !

    Scalar k1 = 2 * std::real(phi_2_0 + phi_2_p2 + phi_2_m2);
    Scalar k2 = 2 * std::real(phi_2_0 - phi_2_p2 - phi_2_m2);

    if(std::abs(k2) > std::abs(k1)) std::swap(k1,k2);

    // reset to original scale
    k1 /= s;
    k2 /= s;

    const Scalar H = 0.5 * (k1 + k2);

    return PointWiseEstimationData(k1, k2, H, N, Vector3::Constant(666), Vector3::Constant(666), nei_count);
}

} // namespace pcp
