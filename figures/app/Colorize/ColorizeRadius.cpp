#include <PCP/Common/Option.h>
#include <PCP/Common/Log.h>
#include <PCP/Common/Colors.h>

#include <PCP/Geometry/Geometry.h>
#include <PCP/Geometry/PLY.h>

#include <PCP/SpacePartitioning/KdTree.h>

using namespace pcp;

int main(int argc, char *argv[])
{
    Option opt(argc, argv);
    const String in_input  = opt.get_string("input",   "i"  ).set_required().set_brief("Input PLY file");
    const String in_output = opt.get_string("output",  "o"  ).set_default("output");
    const Scalar in_radius = opt.get_float( "radius",  "r"  ).set_default(0.01).set_brief("Factor of the AABB diagonal length");
    const int    in_idx    = opt.get_int(   "index",   "idx").set_default(0);
    const Scalar in_eps    = opt.get_float( "epsilon", "eps").set_default(0).set_brief("Displacment as a factor of the radius");

    bool ok = opt.ok();
    if(!ok) return 1;
    info() << opt;

    Geometry pts;
    ok = PLY::load(in_input, pts);
    if(!ok) return 1;
    pts.request_colors(Colors::Gray());

    if(in_idx < 0 || pts.size() <= in_idx)
    {
        warning() << "Index out of bounds: " << in_idx << " not in (0," << pts.size() << "(";
        return 0;
    }

    const Aabb   aabb = pts.aabb();
    const Scalar aabb_diag = aabb.diagonal().norm();
    const Scalar radius = in_radius * aabb_diag;
    info() << "Aabb   = " << aabb.diagonal().norm();
    info() << "radius = " << radius;

    pts.build_kdtree();

    const auto colomap = Colormap::Hot();

    int nei_count = 0;
    for(int j : pts.kdtree().range_neighbors(in_idx, radius))
    {
        const Scalar d = (pts[in_idx] - pts[j]).norm() / radius;
        const Scalar w = (d*d - 1) * (d*d - 1);
        pts.color(j) = colomap(w);
        ++nei_count;
    }
    info() << "Neighbor count = " << nei_count;

    pts.color(in_idx) = Colors::Blue();

    if(pts.has_normals())
    {
        pts[in_idx] += in_eps * radius * pts.normal(in_idx);
    }

    PLY::save(in_output+".ply", pts);

    return 0;
}
