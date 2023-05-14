//
// Created by qzj on 2021/4/25.
//

#ifndef SRC_CONFIG_H
#define SRC_CONFIG_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <glog/logging.h>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

namespace config {

    // benchmark config
    extern std::string config_file;
    extern std::string algorithm;
    extern std::string kitti_root;
    extern std::string label_dir;

    //downsample:
    extern std::string downsample_method;
    extern double leaf_size;
    extern double random_ratio;

    extern int normal_neighbor_num;
    extern int min_cluster_size;
    extern int max_cluster_size;

    // ransac
    extern int max_iteration;
    extern double distance_threshold;

    // region growing
    extern double normal_consistency;
    extern double curvature_threshold;
    extern int region_neighbor_num;

    void readParameters(std::string config_file);

}

class TicToc {
public:
    TicToc() {
        tic();
        duration_ms = 0;
        duration_s = 0;
    }

    void tic() {
        start = std::chrono::steady_clock::now();
    }

    double toc() {
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        duration_s = elapsed_seconds.count();
        duration_ms = elapsed_seconds.count() * 1000;
        return duration_s;
    }
    
    double time_used() {
        return duration_s;
    }
    
    double time_used_ms() {
        return duration_ms;
    }
    
private:
    std::chrono::steady_clock::time_point start, end;
    double duration_s, duration_ms;
};

#endif //SRC_CONFIG_H
