#include <PCP/Common/Option.h>
#include <PCP/Common/Log.h>

#include <PCP/Curvature/GlobalErrorData.h>
#include <PCP/Curvature/GlobalStatisticsData.h>

using namespace pcp;

int main(int argc, char** argv)
{
    Option opt(argc, argv);
    const String in_input  = opt.get_string("input",  "i").set_required();
    const String in_output = opt.get_string("output", "o" ).set_default("output");

    bool ok = opt.ok();
    if(!ok) return 1;
    info() << opt;

    GlobalErrorData errors;
    ok = errors.load(in_input);
    if(!ok) return 1;

    GlobalStatisticsData stats(errors);
    stats.save(in_output);

    return 0;
}


