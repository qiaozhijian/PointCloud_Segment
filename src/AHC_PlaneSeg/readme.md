+AHC方法论文：Fast plane extraction in organized point clouds using agglomerative hierarchical clustering
+方法源代码https://github.com/yanyan-li/PlanarSLAM/blob/master/src/Frame.cc#L92

## 准备工作 
将PlaneExtract.h 和 peac 文件夹放在 include文件中，PlaneExtractor.cpp 放在 src文件夹中
在.h文件中include"PlaneExtractor.h",并在类中定义成员 PlaneDetection planeDetector。

## 调用方法 
planeDetector.readDepthImage(cv::Mat depthImg, cv::Mat &K);读取深度图像，K为相机内参
bool readColorImage(cv::Mat RGBImg);读取彩色图像
planeDetector.runPlaneDetection();计算平面


##输出
auto extractedPlane planeDetector.plane_filter.extractedPlanes[ind]获取第ind个平面
extractedPlane->normal[0];//nx平面法向量
extractedPlane->normal[1];//ny
extractedPlane->normal[2];//nz
extractedPlane->center[0];//cx平面中心点
extractedPlane->center[1];//cy
extractedPlane->center[2];//cz
d = (float) -(NCD.position.x*NCD.orientation.x +NCD.position.y*NCD.orientation.y +NCD.position.z*NCD.orientation.z);//到相机距离d = - (nx*cx+ny*cy+nz*cz)

