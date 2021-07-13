#include <PCP/Common/Option.h>
#include <PCP/Common/Log.h>

#include <PCP/Geometry/Geometry.h>
#include <PCP/Geometry/orthonormal_basis.h>

#include <PCP/Curvature/GlobalCurvatureData.h>
#include <PCP/Curvature/CustomLoader.h>

#include <random>

using namespace pcp;

int main(int argc, char** argv)
{
    Option opt(argc, argv);
    const String in_input       = opt.get_string("input",  "i").set_required();
    const String in_output      = opt.get_string("output", "o").set_default("output");
    const Scalar in_stddev_pos  = opt.get_float( "pos"        ).set_default(0).set_brief("factor of aabb diag");
    const Scalar in_stddev_nor  = opt.get_float( "nor"        ).set_default(0).set_brief("in degree");
    const bool   in_fixed       = opt.get_bool ( "fixed"      ).set_default(false).set_brief("normals pertured in a fixed direction (1,0,1)");
    const int    in_seed        = opt.get_int  ( "seed"       ).set_default(0);

    bool ok = opt.ok();
    if(!ok) return 1;
    info() << opt;

    Scalar seed = 0;
    if(in_seed >= 0)
    {
        seed = in_seed;
        info() << "Given seed = " << seed;
    }
    else
    {
        seed = time(nullptr);
        info() << "Random seed = " << seed;
    }
    srand(seed);

    const bool noise_on_pos = in_stddev_pos != 0;
    const bool noise_on_nor = in_stddev_nor != 0;
    if(in_fixed && !noise_on_nor) warning() << "'fixed' is given but 'nor' is 0...";

    Geometry points;
    GlobalCurvatureData groundtruth;
    ok = CustomLoader::Load(in_input, points, groundtruth);
    if(!ok) return 1;
    const int point_count = points.size();

    const Scalar aabb_diag   = points.aabb_diag();
    const Scalar std_dev_pos = in_stddev_pos * aabb_diag;
    const Scalar std_dev_nor = in_stddev_nor;
    info() << "Info:";
    info() << "  aabb_diag   = " << aabb_diag  ;
    info() << "  std_dev_pos = " << std_dev_pos;
    info() << "  std_dev_nor = " << std_dev_nor;
    info() << "Noise on:";
    info() << "  positions   = " << (noise_on_pos ? "yes":"no");
    info() << "  normals     = " << (noise_on_nor ? "yes":"no");

    std::default_random_engine gen;
    std::normal_distribution<double> dist_pos(0, std_dev_pos);
    std::normal_distribution<double> dist_nor(0, std_dev_nor / 180 * M_PI); // in radian
    std::uniform_real_distribution<double> dist_01(0,1);

    for(int i=0; i<point_count; ++i)
    {
        if(noise_on_pos)
        {
            const Scalar dx = dist_pos(gen);
            const Scalar dy = dist_pos(gen);
            const Scalar dz = dist_pos(gen);
            points[i] += Vector3(dx,dy,dz);
        }

        if(noise_on_nor)
        {
            Vector3 u, v;
            orthonormal_basis(u, v, points.normal(i));
            const Scalar angle = dist_nor(gen);

            Vector3 axis;
            if(in_fixed)
            {
                axis = Vector3(1,0,1).normalized(); // arbitrary
            }
            else
            {
                const Scalar theta = 2*M_PI * dist_01(gen); // random angle in (0,2*pi)
                axis = std::cos(theta) * u + std::sin(theta) * v;
            }
            Eigen::AngleAxis<Scalar> R(angle, axis);
            points.normal(i) = R * points.normal(i);
        }
    }

    CustomLoader::Save(in_output, points, groundtruth);

    return 0;
}

