#pragma once

#include <PCP/Curvature/MethodProperties.h>
#include <PCP/Curvature/PointWiseEstimationData.h>

#include <string>
#include <memory>

namespace pcp {

class Geometry;
class GlobalCurvatureData;
class GlobalEstimationData;

constexpr Scalar SUM_WEIGHT_MIN = 0.00001;
constexpr int    NEI_COUNT_MIN  = 8;
constexpr int    MLS_STEP_MAX   = 20;
constexpr Scalar MLS_EPSILON    = 0.001;

class AbstractPointWiseCurvatureComputer
{
public:
    virtual MethodProperties method_properties() const = 0;
    virtual PointWiseEstimationData compute(const Geometry& points, int i, Scalar r) const = 0;
};

#define OVERRIDDEN_METHODS public: MethodProperties method_properties() const override; PointWiseEstimationData compute(const Geometry& points, int i, Scalar r) const override;

class CurvatureComputer_Barycenter : public AbstractPointWiseCurvatureComputer{OVERRIDDEN_METHODS};
class CurvatureComputer_PCAPlane   : public AbstractPointWiseCurvatureComputer{OVERRIDDEN_METHODS};
class CurvatureComputer_APSS       : public AbstractPointWiseCurvatureComputer{OVERRIDDEN_METHODS};
class CurvatureComputer_ASO        : public AbstractPointWiseCurvatureComputer{OVERRIDDEN_METHODS};
class CurvatureComputer_OJets      : public AbstractPointWiseCurvatureComputer{OVERRIDDEN_METHODS};
class CurvatureComputer_WJets      : public AbstractPointWiseCurvatureComputer{OVERRIDDEN_METHODS};
class CurvatureComputer_PSS        : public AbstractPointWiseCurvatureComputer{OVERRIDDEN_METHODS};

class GlobalCurvatureComputer
{
public:
    template<typename T>
    static GlobalCurvatureComputer make()
    {
        GlobalCurvatureComputer c;
        c.m_computer = std::make_shared<T>();
        return c;
    }

    void compute(const Geometry& points,
                 Scalar r,
                 GlobalEstimationData& estimations,
                 bool prog = false);

public:
    std::shared_ptr<AbstractPointWiseCurvatureComputer> m_computer;
};

} // namespace pcp
