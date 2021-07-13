#include <PCP/Common/Option.h>
#include <PCP/Common/Log.h>
#include <PCP/Common/Timer.h>

#include <PCP/Geometry/Geometry.h>

#include <PCP/Curvature/CustomLoader.h>
#include <PCP/Curvature/Curvature.h>
#include <PCP/Curvature/GlobalCurvatureData.h>
#include <PCP/Curvature/GlobalEstimationData.h>
#include <PCP/Curvature/Methods/VCM.h>

using namespace pcp;

int main(int argc, char** argv)
{
    Timer main_timer;

    Option opt(argc, argv);
    const String in_input  = opt.get_string("input",  "i").set_required();
    const String in_output = opt.get_string("output", "o").set_default("output");
    const bool   in_prog   = opt.get_bool(  "prog",   "p").set_default(false).set_brief("Show progress for each method");
    const Scalar in_radius = opt.get_float( "radius", "r").set_default(0).set_brief("Factof of AABB diag (except if -abs)");
    const bool   in_abs    = opt.get_bool(  "abs"        ).set_default(0).set_brief("Give the radius as absolute value");

    bool ok = opt.ok();
    if(!ok) return 1;
    info() << opt;

    std::vector<GlobalCurvatureComputer> computers;
    computers.push_back(GlobalCurvatureComputer::make<CurvatureComputer_Barycenter>());
    computers.push_back(GlobalCurvatureComputer::make<CurvatureComputer_PCAPlane  >());
    computers.push_back(GlobalCurvatureComputer::make<CurvatureComputer_APSS      >());
    computers.push_back(GlobalCurvatureComputer::make<CurvatureComputer_ASO       >());
    computers.push_back(GlobalCurvatureComputer::make<CurvatureComputer_OJets     >());
    computers.push_back(GlobalCurvatureComputer::make<CurvatureComputer_WJets     >());
    computers.push_back(GlobalCurvatureComputer::make<CurvatureComputer_PSS       >());

    Geometry points;
    GlobalCurvatureData groundtruth;
    ok = CustomLoader::Load(in_input, points, groundtruth);
    if(!ok) return 1;

    const Scalar aabb_diag = points.aabb_diag();
    const Scalar r = in_abs ? in_radius : in_radius * aabb_diag;
    info() << "radius = " << r << " (" << in_radius << " x " << aabb_diag << ") " << (in_abs ? "[absolute value]":"[ratio]");

    points.build_kdtree();

    GlobalEstimationData estimations;

    // Estimations -------------------------------------------------------------
    for(int k=0; k<int(computers.size()); ++k)
    {
        info() << "Method " << k << " (" << computers[k].m_computer->method_properties().name << ")";
        Timer timer;
        {
            computers[k].compute(points, r, estimations, in_prog);
        }
        const auto time_s        = timer.time_sec();
        const int  nei_count     = estimations.average_nei_count();
        const int  invalid_count = estimations.size() - estimations.valid_count();
        const int  invalid_prct  = int(Scalar(invalid_count)/estimations.size()*100);

        info() << "    time      = " << int(time_s) << "s (" << Timer::sec_str(time_s) << ")";
        info() << "    neighbors = " << nei_count;
        info() << "    invalid   = " << invalid_count << "/" << estimations.size() << " (" << invalid_prct << "%)";

        estimations.save(in_output + computers[k].m_computer->method_properties().name + ".txt");
    }

    // VCM ---------------------------------------------------------------------
    info() << "Method " << computers.size() << " (" << "VCM" << ")";
    Timer timer;
    {
        compute_VCM(points, r, estimations);
    }
    const auto time_s        = timer.time_sec();
    const int  nei_count     = estimations.average_nei_count();
    const int  invalid_count = estimations.size() - estimations.valid_count();
    const int  invalid_prct  = int(Scalar(invalid_count)/estimations.size()*100);

    info() << "    time      = " << int(time_s) << "s (" << Timer::sec_str(time_s) << ")";
    info() << "    neighbors = " << nei_count;
    info() << "    invalid   = " << invalid_count << "/" << estimations.size() << " (" << invalid_prct << "%)";


    estimations.save(in_output + "VCM" + ".txt");


    const auto main_time_s = main_timer.time_sec();
    info() << "TOTAL time = " << int(main_time_s) << "s (" << Timer::sec_str(main_time_s) << ")";

    return 0;
}


