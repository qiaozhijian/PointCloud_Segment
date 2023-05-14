#include "planeSeg.h"
#include <pcl/filters/random_sample.h>
#include "boost/thread.hpp"
#include "patchwork/patchwork.hpp"

PlaneSegment::PlaneSegment(bool visualize) {
    _cloud.reset(new pcl::PointCloud<PointType>());
    _cloud->clear();

    _cloudDownSample.reset(new pcl::PointCloud<PointType>());
    _cloudDownSample->clear();

    _cloudNormal.reset(new pcl::PointCloud<pcl::Normal>());
    _cloudNormal->clear();

    _colored_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    _colored_cloud->clear();

    _bVisual = visualize;
}


void PlaneSegment::VoxelDownSample() {
    if (config::downsample_method == "voxelgrid") {
        pcl::VoxelGrid<PointType> vg;
        cout << "before filtering, point cloud->size: " << _cloud->points.size() << endl;
        vg.setInputCloud(_cloud);
        vg.setLeafSize(config::leaf_size, config::leaf_size, config::leaf_size);
        vg.filter(*_cloudDownSample);
        cout << "After filtering, point cloud->size: " << _cloudDownSample->points.size() << endl;
    } else if (config::downsample_method == "random") {
        pcl::RandomSample<PointType> rs;
        cout << "before filtering, point cloud->size: " << _cloud->points.size() << endl;
        rs.setInputCloud(_cloud);
        int num = (int) (_cloud->points.size() * config::random_ratio);
        rs.setSample(num);
        rs.filter(*_cloudDownSample);
        cout << "After filtering, point cloud->size: " << _cloudDownSample->points.size() << endl;
    } else if (config::downsample_method == "none") {
        _cloudDownSample = _cloud;
    } else if (config::downsample_method == "patchwork"){
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloudI(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*_cloud, *ptrCloudI);
        pcl::PointCloud<pcl::PointXYZI> srcGround;
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptrSrcNonground(new pcl::PointCloud<pcl::PointXYZI>);
        double tSrc = 0.0;
        PatchWork<pcl::PointXYZI> patchwork("KITTI");
        patchwork.estimate_ground(*ptrCloudI, srcGround, *ptrSrcNonground, tSrc);
        pcl::copyPointCloud(*ptrSrcNonground, *_cloudDownSample);
    } else {
        cerr << "Wrong downsample method!" << endl;
        exit(1);
    }
}

void PlaneSegment::SegOnePlane(PointCloudPtr &cloudIn, PointCloudPtr &cloudPlane) {
    // Euclidean Segment
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(config::max_iteration);
    //    查询点到目标模型的距离阈值（如果大于此阈值，则查询点不在目标模型上，默认值为0）
    seg.setDistanceThreshold(config::distance_threshold);        //阀值
    // 从剩余点云中再分割出当前最大平面分量
    seg.setInputCloud(cloudIn);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        cerr << "Could not estimate a planar model for the given dataset." << endl;
        return;
    }

    // 提取平面模型内点
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloudIn);
    extract.setIndices(inliers);
    extract.setNegative(false);
    cloudPlane->clear();
    extract.filter(*cloudPlane);

    // 过滤平面点，得到剩余点云
    extract.setNegative(true);
    extract.filter(*cloudIn);

}

void PlaneSegment::EuclideanSegment() {

    int nr_points = _cloudDownSample->points.size();  //降采样后剩余点云数量
    int r, g, b;

    PointCloudPtr cloud_plane(new pcl::PointCloud<PointType>);
    int planes = 0;
    while (cloud_plane->size() > config::min_cluster_size or planes == 0)  //现：提取的当前平面点数>阈值
    {
        SegOnePlane(_cloudDownSample, cloud_plane);
        planes++;

        if (!_bVisual)
            continue;
        //复制当前平面至最终结果
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;
        for (int k = 0; k < cloud_plane->size(); k++) {
            pcl::PointXYZRGB point;
            point.x = cloud_plane->points[k].x;
            point.y = cloud_plane->points[k].y;
            point.z = cloud_plane->points[k].z;
            point.r = r;
            point.g = g;
            point.b = b;
            _colored_cloud->points.push_back(point);
        }
    }
}

void PlaneSegment::Visualize() {
    // Visualize
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(_colored_cloud, "sample cloud");
    viewer->addCoordinateSystem(0.2);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    }
}

void PlaneSegment::RegionGrowSeg() {
    //设置搜索结构
    pcl::search::Search<PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType>>(
            new pcl::search::KdTree<PointType>);
    //聚类对象<点，法线>
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(config::min_cluster_size);         //最小聚类点数
    reg.setMaxClusterSize(config::max_cluster_size);    //最大
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(config::region_neighbor_num);     //邻域点个数，决定这是一个平面(决定容错率，设置大时有倾斜也可接受，设置小时检测到的平面会很小）
    reg.setInputCloud(_cloudDownSample);
    reg.setInputNormals(_cloudNormal);
    reg.setSmoothnessThreshold(config::normal_consistency);    //平滑阈值_两法线偏差允许范围，若小于该阈值则认为两点属于同一聚类
    reg.setCurvatureThreshold(config::curvature_threshold);                    //曲率阀值_两点曲率偏差允许范围，若满足前一条件并小于该阈值则采用新增加点迭代进行区域生长

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    if (_bVisual)
        _colored_cloud = reg.getColoredCloud();
}


void PlaneSegment::SegOnePlaneWithNormal(PointCloudPtr &cloudIn, pcl::PointCloud<pcl::Normal>::Ptr nomalIn,
                                         PointCloudPtr &cloudPlane, pcl::PointCloud<pcl::Normal>::Ptr nomalPlane) {

    // ****************为平面模型创建分割对象并设置所有参数
    pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;    //分割对象
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);//设置模型类型
    seg.setMethodType(pcl::SAC_RANSAC);//设置随机采样一致性方法类型
    seg.setNormalDistanceWeight(0.1);  // 法线信息权重
    seg.setMethodType(pcl::SAC_RANSAC);//随机采样一致性算法
    seg.setMaxIterations(config::max_iteration);         //最大迭代次数
    seg.setDistanceThreshold(config::distance_threshold);    //设置内点到模型的距离允许最大值
    seg.setInputCloud(cloudIn); //输入点云
    seg.setInputNormals(nomalIn);//设置输人点云的法线，normals为指向法线的指针。
    seg.segment(*inliers_plane, *coefficients_plane);//存储分割结果到点几何inliers_plane及存储平面模型的系数coefficients_plane

    // *************从点云中抽取分割的处在平面上的点集
    pcl::ExtractIndices<PointType> extract;      //// 创建滤波器对象（点提取对象）
    extract.setInputCloud(cloudIn);
    extract.setIndices(inliers_plane);//设置分割后的点内为需要提取的点集
    extract.setNegative(false);//设置提取点内而非点外。
    extract.filter(*cloudPlane);// 存储分割得到的平面上的点到点云文件
    extract.setNegative(true);//设置提取点外而非点内。
    extract.filter(*cloudIn);//提取输出并储存到cloudLeavex

    // ************Remove the planar inliers, extract the rest
    pcl::ExtractIndices<pcl::Normal> extract_normals;    //点提取对象
    extract_normals.setInputCloud(nomalIn);
    extract_normals.setIndices(inliers_plane);
    extract_normals.setNegative(false);
    extract_normals.filter(*nomalPlane);
    extract_normals.setNegative(true);
    extract_normals.filter(*nomalIn);

}

void PlaneSegment::EuclideanNormalSeg() {

    int r, g, b;
    PointCloudPtr cloud_plane(new pcl::PointCloud<PointType>);
    pcl::PointCloud<pcl::Normal>::Ptr nomalPlane(new pcl::PointCloud<pcl::Normal>);
    int planes = 0;
    while (cloud_plane->size() > config::min_cluster_size or planes == 0)  //现：提取的当前平面点数>阈值
    {
        SegOnePlaneWithNormal(_cloudDownSample, _cloudNormal, cloud_plane, nomalPlane);
        planes++;

        if (!_bVisual)
            continue;
        //复制当前平面至最终结果
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;
        for (int k = 0; k < cloud_plane->size(); k++) {
            pcl::PointXYZRGB point;
            point.x = cloud_plane->points[k].x;
            point.y = cloud_plane->points[k].y;
            point.z = cloud_plane->points[k].z;
            point.r = r;
            point.g = g;
            point.b = b;
            _colored_cloud->points.push_back(point);
        }
    }
    LOG(INFO) << "The number of planes is " << planes << endl;
}

void PlaneSegment::EstimateNormal() {
    clock_t startTime, endTime;
    startTime = clock();
    // ************点云进行法线估计，为后续进行基于法线的分割准备数据
    pcl::NormalEstimation<PointType, pcl::Normal> ne;  //法线估计对象
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);//设置内部算法实现时所用的搜索对象，tree为指向kdtree或者octree对应的指针
    ne.setInputCloud(_cloudDownSample);
    ne.setKSearch(config::normal_neighbor_num);// 设置K近邻搜索时所用的K参数
    ne.compute(*_cloudNormal);//计算特征值
    endTime = clock();
    cout << "The NormalEstimation time is:" << (double) (endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
}

void PlaneSegment::ReadData(std::string &in_file) {

    if (pcl::io::loadPCDFile<PointType>(in_file, *_cloud) == -1) {
        cerr << ("Couldn't read file " + in_file + " \n");
    }
    _cloud = kitti_loader::GetCloud(in_file);

    clock_t startTime_ds, endTime_ds;
    startTime_ds = clock();
    VoxelDownSample();
    endTime_ds = clock();
    cout << "The downsample time is:" << (double) (endTime_ds - startTime_ds) / CLOCKS_PER_SEC << "s" << endl;

    if (config::algorithm == "ransac_normal" or config::algorithm == "region_growing") {
        EstimateNormal();
    }
}

void PlaneSegment::SegmentPlanes() {

    clock_t startTime, endTime;
    startTime = clock();
    if (config::algorithm == "ransac") {
        EuclideanSegment();
    } else if (config::algorithm == "ransac_normal") {
        EuclideanNormalSeg();
    } else if (config::algorithm == "region_growing") {
        RegionGrowSeg();
    } else {
        cerr << "Wrong algorithm name!" << endl;
    }
    endTime = clock();
    cout << "The run time is:" << (double) (endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    if (_bVisual)
        Visualize();
}