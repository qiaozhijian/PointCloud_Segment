#include "include/PlaneExtractor.h"
//#include "include/planeSeg.h"
#include "global_defination.h"
#include <opencv/cv.h>
#include <vector>

// fixme 根据照片大小不同需要修改PlaneExtractor.h中的kDepthWidth，kDepthHeight
// kScaleFactor需要自己调一个合适的数值
int main(int argc, char **argv) {

    int height = 480;
    int weight = 848;
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0, 0) = 902.476;//旋转
    K.at<float>(1, 1) = 901.800;
    K.at<float>(0, 2) = 637.571;//平移
    K.at<float>(1, 2) = 363.211;
    std::string img_name = "/home/fjy/Desktop/YunXiao_master/PointCloud_Segment/data/frame0020.jpg";
    cv::Mat in_image = cv::imread(img_name);//读取的照片是RGBD的深度图
    if (in_image.empty()) {
        //检查是否读取图像
        std::cout << "Error! Input image cannot be read...\n";
        return -1;
    }

//    imwrite(img_name, img);
    in_image =in_image(cv::Rect(0,0,840,480));

    //输入的深度图需要是CV_16U格式
    if (in_image.type() != CV_16U) {
        in_image.convertTo(in_image, CV_16U, 1);
    }
    // Load data points
    PlaneDetection planeDetector;
    planeDetector.readDepthImage(in_image,K);
    planeDetector.runPlaneDetection();

    std::vector<int> max_N_ind;
    for (int i = 0; i < planeDetector.plane_num_; i++) {
        auto extractedPlane = planeDetector.plane_filter.extractedPlanes[i];
        max_N_ind.push_back(extractedPlane->N);

    }

    auto biggest = max_element(max_N_ind.begin(),max_N_ind.end());
    auto biggest_ind = distance(max_N_ind.begin(),biggest);
    auto extractedPlane = planeDetector.plane_filter.extractedPlanes[biggest_ind];

    //n最大面的法向量，c最大面的中心点
    double nx = extractedPlane->normal[0];
    double ny = extractedPlane->normal[1];
    double nz = extractedPlane->normal[2];
    double cx = extractedPlane->center[0];
    double cy = extractedPlane->center[1];
    double cz = extractedPlane->center[2];
    float d = (float) -(nx * cx + ny * cy + nz * cz);
//    PlaneSegment plane_segment(segment, downsample, true);
//    plane_segment.ReadData(filename);
//    plane_segment.SegmentPlanes();


    return 0;
}
