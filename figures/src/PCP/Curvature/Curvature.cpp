#include <PCP/Curvature/Curvature.h>
#include <PCP/Curvature/GlobalEstimationData.h>

#include <PCP/Geometry/Geometry.h>

#include <PCP/Common/Progress.h>
#include <PCP/Common/Macro.h>

namespace pcp {

// -------->                                                                      Name          ori.     H     N      k    k_si.   dir
MethodProperties CurvatureComputer_PSS       ::method_properties() const {return {"PSS",        false, true, true,  true,  true,  true };}
MethodProperties CurvatureComputer_OJets     ::method_properties() const {return {"OJets",      false, true, true,  true,  true,  true };}
MethodProperties CurvatureComputer_WJets     ::method_properties() const {return {"WJets",      false, true, true,  true,  true,  true };}
MethodProperties CurvatureComputer_Barycenter::method_properties() const {return {"Barycenter", false, true, false, false, false, false};}
MethodProperties CurvatureComputer_PCAPlane  ::method_properties() const {return {"PCAPlane",   false, true, true,  false, false, true };}
MethodProperties CurvatureComputer_APSS      ::method_properties() const {return {"APSS",       true,  true, true,  false, false, false};}
MethodProperties CurvatureComputer_ASO       ::method_properties() const {return {"ASO",        true,  true, true,  true,  true,  true };}

void GlobalCurvatureComputer::compute(const Geometry& points,
                                      Scalar r,
                                      GlobalEstimationData& estimations,
                                      bool b)
{
    estimations.resize(points.size());

    auto prog = Progress(points.size(), b);

    #pragma omp parallel for
    for(int i=0; i<points.size(); ++i)
    {
        estimations[i] = m_computer->compute(points, i, r);
        ++prog;
    }
}

} // namespace pcp
