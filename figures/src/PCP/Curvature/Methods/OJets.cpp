#include <PCP/Curvature/Curvature.h>
#include <PCP/Geometry/Geometry.h>

#include <PCP/SpacePartitioning/KdTree.h>

// CGAL ------------------------------------------------------------------------
#define CGAL_DISABLE_ROUNDING_MATH_CHECK // otherwise valgrind dont work
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Monge_via_jet_fitting.h>
typedef pcp::Scalar                              DFT;
typedef CGAL::Simple_cartesian<DFT>              Data_Kernel;
typedef Data_Kernel::Point_3                     DPoint;
typedef CGAL::Monge_via_jet_fitting<Data_Kernel> My_Monge_via_jet_fitting;
typedef My_Monge_via_jet_fitting::Monge_form     My_Monge_form;

namespace pcp {

PointWiseEstimationData CurvatureComputer_OJets::compute(const Geometry& points, int i, Scalar r) const
{
    constexpr size_t d_fitting = 2;
    constexpr size_t d_monge = 2;

    // fct parameters
    My_Monge_form monge_form;
    My_Monge_via_jet_fitting monge_fit;

    int nei_count = 0;

    std::vector<DPoint> in_points;
    for(int j : points.kdtree().range_neighbors(i, r))
    {
        in_points.push_back(DPoint(points[j].x(), points[j].y(), points[j].z()));
        ++nei_count;
    }

    if(nei_count < NEI_COUNT_MIN) return PointWiseEstimationData::Invalid();

    monge_form = monge_fit(in_points.begin(), in_points.end(), d_fitting, d_monge);

    // re-orient the quadric
    const auto true_normal = My_Monge_form::Vector_3(points.normal(i)[0],points.normal(i)[1],points.normal(i)[2]);
    monge_form.comply_wrt_given_normal(true_normal);

    const Vector3 N = Vector3(monge_form.normal_direction().x(),
                              monge_form.normal_direction().y(),
                              monge_form.normal_direction().z());

    Scalar k1 = monge_form.principal_curvatures(0);
    Scalar k2 = monge_form.principal_curvatures(1);

    if(std::abs(k2) > std::abs(k1)) std::swap(k1,k2);

    // CGAL convention: +1 = concave and -1 = convex
    // we want the inverse
    k1 *= -1;
    k2 *= -1;

    const Vector3 dir1 = Vector3(monge_form.maximal_principal_direction().x(),
                                 monge_form.maximal_principal_direction().y(),
                                 monge_form.maximal_principal_direction().z());
    const Vector3 dir2 = Vector3(monge_form.minimal_principal_direction().x(),
                                 monge_form.minimal_principal_direction().y(),
                                 monge_form.minimal_principal_direction().z());

    const Scalar H = 0.5 * (k1 + k2);

    return PointWiseEstimationData(k1, k2, H, N, dir1, dir2, nei_count);
}

} // namespace pcp
