#include <PCP/Common/Option.h>
#include <PCP/Common/Log.h>
#include <PCP/Common/Colors.h>

#include <PCP/Geometry/Geometry.h>
#include <PCP/Geometry/PLY.h>

#include <PCP/Curvature/CustomLoader.h>
#include <PCP/Curvature/GlobalCurvatureData.h>

using namespace pcp;

inline Vector4 colorize(float value, float limit)
{
         if(value < 0)          return Colors::Blue();
    else if(value < 0.01*limit) return Colors::Green();
    else if(value > 0.5)        return Colors::Gray();
    else {
        const auto t = value/limit;
        return (1-t) * Colors::White() + t * Colors::Red();
    }
}

int main(int argc, char *argv[])
{
    Option opt(argc, argv);
    const String in_input  = opt.get_string("input",  "i").set_required();
    const String in_output = opt.get_string("output", "o").set_default("output");
    const bool   in_abs    = opt.get_bool(  "abs"        ).set_default(false);
    const Scalar in_lim    = opt.get_float( "limit",  "l").set_default(0);

    bool ok = opt.ok();
    if(!ok) return 1;
    info() << opt;

    Geometry points;
    GlobalCurvatureData data;
    ok = CustomLoader::Load(in_input, points, data);
    if(!ok) return 1;
    const int point_count = points.size();


    Scalar lim = in_lim;
    if(lim == 0)
    {
        std::vector<Scalar> H(point_count);
        for(int i=0; i<point_count; ++i) H[i] = data[i].H();
        std::sort(H.begin(), H.end(), [](Scalar a, Scalar b){return std::abs(a) < std::abs(b);});
        lim = H[0.90 * (point_count-1)];
    }
    info() << "limit = " << lim;

    points.request_colors();
    if(in_abs) for(int i=0; i<point_count; ++i) points.color(i) = colorize(std::abs(data[i].H()), lim);
    else       for(int i=0; i<point_count; ++i) points.color(i) = colorize(         data[i].H() , lim);

    PLY::save(in_output + "_H.ply", points);

    for(int i=0; i<point_count; ++i) points.color(i) = Colors::Normal(data[i].normal());
    PLY::save(in_output + "_n.ply", points);

    return 0;
}
