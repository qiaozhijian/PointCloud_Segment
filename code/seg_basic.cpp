
#include "include/seg.h"
#include "include/datapretreat.h"



int main (int argc, char** argv)
{
    clock_t startTime,endTime;
    startTime = clock();

    // Load data points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string seq = "01";
    string filename = "/home/qsy-5208/Documents/PointCloud_Segment/global_pcs/secen_pcd"+seq+".pcd";
    datapretreat d;
    d.ReadData(filename, cloud);


    // PointCloud Segment
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;    //分割对象

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); //模型类型_平面
    seg.setMethodType (pcl::SAC_RANSAC);		   //设置方法【聚类或随机样本一致性】
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud->makeShared ());
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return -1;
    }

    cerr << "Model coefficients: " << coefficients->values[0] << " "
              <<coefficients->values[1] << " "
              <<coefficients->values[2] << " "
              <<coefficients->values[3] <<std::endl;    //模型系数
    cerr << "Model inliers: " << inliers->indices.size () << endl;    //估计平面模型过程中使用的内点


    // Extract Indices
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    cerr << "After extracting the planar inliers, point cloud->size: " << cloud_filtered->size() << endl;


    // Save pcd
    pcl::PCDWriter writer;
    writer.write ("/home/qsy-5208/Documents/PointCloud_Segment/result/basic"+seq+".pcd", *cloud_filtered, false);


    endTime = clock();
    cout << "The run time is:" <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    return 0;
}