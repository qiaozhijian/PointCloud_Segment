//
// Created by qzj on 2021/12/24.
//

#include "config.h"
#include "glog/logging.h"

namespace config {

    // benchmark config
    std::string config_file = "";
    std::string algorithm = "";
    std::string kitti_root = "";
    std::string label_dir = "";

    //downsample:
    std::string downsample_method = "";
    double leaf_size = 0.1;
    double random_ratio = 0.1;

    int normal_neighbor_num = 20;
    int min_cluster_size = 100;
    int max_cluster_size = 100000;

    // ransac
    int max_iteration = 1000;
    double distance_threshold = 0.1;

    // region growing
    double normal_consistency = 5.0;
    double curvature_threshold = 1.0;
    int region_neighbor_num = 20;


    void readParameters(std::string config_file_) {
        config_file = config_file_;
        std::ifstream fin(config_file);
        if (!fin) {
            std::cout << "config_file: " << config_file << " not found." << std::endl;
            return;
        }
        YAML::Node config_node = YAML::LoadFile(config_file);
        algorithm = config_node["algorithm"].as<std::string>();
        LOG(INFO) << "algorithm: " << algorithm;
        kitti_root = config_node["dataset"]["dataset_path"].as<std::string>();
        LOG(INFO) << "kitti_root: " << kitti_root;
        label_dir = config_node["dataset"]["label_dir"].as<std::string>();
        LOG(INFO) << "label_dir: " << label_dir;

        downsample_method = config_node["downsample"]["method"].as<std::string>();
        LOG(INFO) << "downsample_method: " << downsample_method;
        leaf_size = config_node["downsample"]["leaf_size"].as<double>();
        LOG(INFO) << "leaf_size: " << leaf_size;
        random_ratio = config_node["downsample"]["random_ratio"].as<double>();
        LOG(INFO) << "random_ratio: " << random_ratio;

        normal_neighbor_num = config_node["normal_neighbor_num"].as<int>();
        LOG(INFO) << "normal_neighbor_num: " << normal_neighbor_num;
        min_cluster_size = config_node["min_cluster_size"].as<int>();
        LOG(INFO) << "min_cluster_size: " << min_cluster_size;
        max_cluster_size = config_node["max_cluster_size"].as<int>();
        LOG(INFO) << "max_cluster_size: " << max_cluster_size;

        max_iteration = config_node["max_iterations"].as<int>();
        LOG(INFO) << "max_iteration: " << max_iteration;
        distance_threshold = config_node["distance_threshold"].as<double>();
        LOG(INFO) << "distance_threshold: " << distance_threshold;

        normal_consistency = config_node["normal_consistency"].as<double>();
        LOG(INFO) << "normal_consistency: " << normal_consistency;
        curvature_threshold = config_node["curvature_threshold"].as<double>();
        LOG(INFO) << "curvature_threshold: " << curvature_threshold;
        region_neighbor_num = config_node["region_neighbor_num"].as<int>();
        LOG(INFO) << "region_neighbor_num: " << region_neighbor_num;
    }
}