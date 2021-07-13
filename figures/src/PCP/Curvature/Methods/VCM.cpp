#include <PCP/Curvature/Methods/VCM.h>
#include <PCP/Curvature/GlobalEstimationData.h>

#include <PCP/Geometry/Geometry.h>

#include <Eigen/Eigenvalues>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/vcm_estimate_normals.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point_3, Vector_3> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;
typedef std::array<double,6> Covariance;

namespace pcp {

void compute_VCM(const Geometry& in_points, Scalar r, GlobalEstimationData& e)
{
    e.resize(in_points.size());

    // convert to CGAL type
    std::vector<PointVectorPair> points(in_points.size());
    #pragma omp parallel for
    for(int i=0; i<in_points.size(); ++i)
    {
        points[i].first  = Point_3( in_points.point(i).x(),  in_points.point(i).y(),  in_points.point(i).z());
        points[i].second = Vector_3(in_points.normal(i).x(), in_points.normal(i).y(), in_points.normal(i).z());
    }

    const Scalar offset_radius = r;
    const Scalar convolution_radius = 0.5 * r;

    std::vector<Covariance> cov;

    CGAL::First_of_pair_property_map<PointVectorPair> point_map;
    const auto np = CGAL::parameters::point_map(point_map).geom_traits(Kernel());

    CGAL::compute_vcm(points, cov, offset_radius, convolution_radius, np);

    #pragma omp parallel for
    for(int i=0; i<in_points.size(); ++i)
    {
        const Covariance& a = cov[i];
        Matrix3 C;
        C << a[0], a[1], a[2],
             a[1], a[3], a[4],
             a[2], a[4], a[5];

        Eigen::SelfAdjointEigenSolver<Matrix3> solver(C);
        const Scalar  l2   = solver.eigenvalues()[0];
        const Scalar  l1   = solver.eigenvalues()[1];
        const Vector3 dir2 = solver.eigenvectors().col(0);
        const Vector3 dir1 = solver.eigenvectors().col(1);
        const Vector3 N    = solver.eigenvectors().col(2);

        const Scalar k1 = 2 * std::sqrt(l1) / r;
        const Scalar k2 = 2 * std::sqrt(l2) / r;

        const Scalar H = 0.5 * (k1 + k2);

        // arbitrary set to 10 because 0 == error
        // TODO use voronoi diagram
        const int nei_count = 10;

        e[i] = PointWiseEstimationData(k1, k2, H, N, dir1, dir2, nei_count);
    }
}

} // namespace pcp

