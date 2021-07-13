#include <PCP/Common/Option.h>
#include <PCP/Common/Log.h>

#include <PCP/Geometry/Geometry.h>

#include <PCP/Curvature/CustomLoader.h>
#include <PCP/Curvature/GlobalCurvatureData.h>
#include <PCP/Curvature/GlobalEstimationData.h>
#include <PCP/Curvature/GlobalErrorData.h>
#include <PCP/Curvature/compute_error.h>

using namespace pcp;

int main(int argc, char** argv)
{
    Option opt(argc, argv);
    const String in_groundtruth = opt.get_string("groundtruth", "gt").set_required();
    const String in_estimations = opt.get_string("estimations", "e" ).set_required();
    const String in_output      = opt.get_string("output",      "o" ).set_default("output");

    bool ok = opt.ok();
    if(!ok) return 1;
    info() << opt;

    Geometry points;
    GlobalCurvatureData groundtruth;
    ok = CustomLoader::Load(in_groundtruth, points, groundtruth);
    if(!ok) return 1;

    GlobalEstimationData estimations;
    ok = estimations.load(in_estimations);
    if(!ok) return 1;

    PCP_ASSERT(groundtruth.size() == points.size());
    PCP_ASSERT(groundtruth.size() == estimations.size());

    GlobalErrorData errors(groundtruth.size());

    #pragma omp parallel for
    for(int i=0; i<groundtruth.size(); ++i)
    {
        errors[i] = compute_error(groundtruth[i], estimations[i]);
    }

    errors.save(in_output);

    return 0;
}


