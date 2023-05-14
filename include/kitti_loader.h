//
// Created by qzj on 23-2-22.
//

#ifndef SRC_KITTI_LOADER_H
#define SRC_KITTI_LOADER_H

#include <map>
#include <string>
#include <utility>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include "boost/format.hpp"
#include "config.h"

namespace kitti_loader {
    
    inline std::string GetPCDPath(const int &seq, const int &frame_id) {
        std::string filename = (boost::format("%s/%02d/velodyne/%06d.bin") % config::kitti_root % seq % frame_id).str();
        return filename;
    }

    inline std::string GetLabelPath(const int &seq, const int &frame_id) {
        std::string filename = (boost::format("%s/%02d/%s/%06d.label") % config::kitti_root % seq % config::label_dir % frame_id).str();
        return filename;
    }

    inline pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(std::string filename) {
        FILE *file = fopen(filename.c_str(), "rb");
        if (!file) {
            std::cerr << "error: failed to load point cloud " << filename << std::endl;
            return nullptr;
        }

        std::vector<float> buffer(1000000);
        size_t num_points =
                fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
        fclose(file);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->resize(num_points);

        for (int i = 0; i < num_points; i++) {
            auto &pt = cloud->at(i);
            pt.x = buffer[i * 4];
            pt.y = buffer[i * 4 + 1];
            pt.z = buffer[i * 4 + 2];
        }

        return cloud;
    }

    inline pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(const int &seq, const int &frame_id) {
        std::string filename = GetPCDPath(seq, frame_id);
        return GetCloud(filename);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr ReadSemCloud(const int &seq, const int &frame_id) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr sem_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        const std::string cloud_file_name = GetPCDPath(seq, frame_id);
        const std::string label_file_name = GetLabelPath(seq, frame_id);

        std::ifstream fcloud(cloud_file_name.c_str(), std::ios::binary);
        std::ifstream flabel(label_file_name.c_str(), std::ios::binary);
        if (!fcloud) {
            std::cerr << "[Error][KittiParser::ReadSemColorCloud] Point Cloud File '" << cloud_file_name
                      << "' is not found!" << std::endl;
            return 0;
        }
        if (!flabel) {
            std::cerr << "[Error][KittiParser::ReadSemColorCloud] Point Cloud Label File '" << label_file_name
                      << "' is not found!" << std::endl;
            return 0;
        }

        // get length of input file:
        fcloud.seekg(0, fcloud.end);
        int cloud_length = (int) fcloud.tellg();
        fcloud.seekg(0, fcloud.beg);

        int cloud_size = cloud_length / sizeof(float);
        sem_cloud->reserve(cloud_size);

        std::vector<float> point_buffer(cloud_size);
        fcloud.read(reinterpret_cast<char *>(&point_buffer[0]), cloud_size * sizeof(float));
        fcloud.close();

        std::vector<uint32_t> label_buffer(cloud_size);
        flabel.read(reinterpret_cast<char *>(&label_buffer[0]), cloud_size * sizeof(uint32_t));
        flabel.close();

        pcl::PointXYZI point;
        for (uint32_t i = 0; i < point_buffer.size(); i += 4) {
            point.x = point_buffer[i];
            point.y = point_buffer[i + 1];
            point.z = point_buffer[i + 2];
            const uint16_t sem_label = label_buffer[i / 4] & 0xFFFF;
            //const uint16_t inst_label = label_buffer[i / 4] >> 16;
            point.intensity = sem_label;
            sem_cloud->push_back(point);
        }
        return sem_cloud;
    }

    void SaveSemLabel(const int &seq, const int &frame_id, const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud) {
        const std::string label_file_name = GetLabelPath(seq, frame_id);
        std::ofstream flabel(label_file_name.c_str(), std::ios::binary);
        if (!flabel) {
            std::cerr << "[Error][KittiParser::ReadSemColorCloud] Point Cloud Label File '" << label_file_name
                      << "' is not found!" << std::endl;
            return;
        }

        std::vector<uint32_t> label_buffer(cloud->size());
        for (uint32_t i = 0; i < cloud->size(); ++i) {
            label_buffer[i] = cloud->at(i).label;
        }

        flabel.write(reinterpret_cast<char *>(&label_buffer[0]), cloud->size() * sizeof(uint32_t));
        flabel.close();
    }

    inline Eigen::Matrix4d GetTr(int seq) {
        std::string calib_file = boost::str(boost::format("%s/%02d/calib.txt") % config::kitti_root % seq);
        std::fstream f;
        f.open(calib_file, std::ios::in);
        if (!f.is_open()) {
            LOG(FATAL) << "Cannot open calib file: " << calib_file;
        }
        std::string line;
        Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();
        while (std::getline(f, line)) {
            std::stringstream ss(line);
            std::string tag;
            ss >> tag;
            if (tag == "Tr:") {
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        ss >> Tr(i, j);
                    }
                }
            }
        }
        return Tr;
    }


    inline Eigen::Matrix4d getLidarPose(const int &seq, const int &frame_id) {
        std::string pose_file = (boost::format("%s/%02d/poses.txt") % config::kitti_root % seq).str();
        //    read kitti pose txt
        std::fstream f;
        f.open(pose_file, std::ios::in);
        if (!f.is_open()) {
            LOG(FATAL) << "Cannot open pose file: " << pose_file;
        }
        Eigen::Matrix4d Tr = GetTr(seq);
        std::string line;
        int num = 0;
        while (std::getline(f, line)) {
            if (num == frame_id) {
                std::stringstream ss(line);
                Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        ss >> Twc(i, j);
                    }
                }
                Eigen::Matrix4d Twl = Twc * Tr;
                return Twl;
            }
            num++;
        }
        return Eigen::Matrix4d::Identity();
    }
}


#endif //SRC_KITTI_LOADER_H
