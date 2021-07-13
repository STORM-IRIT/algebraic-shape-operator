#pragma once

#include <log.h>
#include <opt.h>

struct Args {
    std::string input;
    int k;
    bool ok;
};

Args parse(int argc, char** argv) {
    opt::Option opt(argc, argv, "Compute and display differential properties "
                                "on oriented point cloud using the Algebraic "
                                "Shape Operator (ASO). Press the up arrow to "
                                "display different properties.");
    const std::string in_input  = opt("input",  "i").set_required().set_brief(
                                  "Input PLY file (3D points and their "
                                  "normal vectors)");
    const int in_k = opt("knn", "k").set_default(20).set_brief(
                     "Nearest neighbors count used to compute the ASO");
    const auto ok = opt.ok();
    return Args{in_input, in_k, ok};
}
