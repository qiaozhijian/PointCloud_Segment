#ifndef POINTCLOUD_SEGMENT_PLANESEG_H
#define POINTCLOUD_SEGMENT_PLANESEG_H

#include <iostream>
#include <cstdlib>
#include <ctime>
#include "kitti_loader.h"

//ransac
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//Region
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

using namespace std;

class PlaneSegment {

public:

    PlaneSegment(bool visualize = false);

    void ReadData(std::string &in_file);

    void VoxelDownSample();

    void SegOnePlane(PointCloudPtr &cloudIn, PointCloudPtr &cloudPlane);

    void SegOnePlaneWithNormal(PointCloudPtr &cloudIn, pcl::PointCloud<pcl::Normal>::Ptr nomalInD,
                               PointCloudPtr &cloudPlane, pcl::PointCloud<pcl::Normal>::Ptr nomalPlane);

    void SegmentPlanes();

    void EuclideanSegment();

    void Visualize();

    void RegionGrowSeg();

    void EuclideanNormalSeg();

    void EstimateNormal();

private:
    PointCloudPtr _cloud;

    PointCloudPtr _cloudDownSample;

    pcl::PointCloud<pcl::Normal>::Ptr _cloudNormal;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _colored_cloud;

    bool _bVisual;
};


#endif //POINTCLOUD_SEGMENT_PLANESEG_H
