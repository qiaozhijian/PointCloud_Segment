#include "include/planeSeg.h"
#include "global_defination.h"
#include "config.h"

int main(int argc, char **argv) {

    string filename = WORK_SPACE_PATH + "/data/000000.bin";

    std::string config_path = WORK_SPACE_PATH + "/configs/params.yaml";
    LOG(INFO) << "config_path: " << config_path;
    LOG(INFO) << "Evaluation: " << config::algorithm;
    config::readParameters(config_path);

    //// Load data points
    PlaneSegment plane_segment(true);
    plane_segment.ReadData(filename);
    plane_segment.SegmentPlanes();

    return 0;
}
