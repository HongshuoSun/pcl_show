#include <iostream>
#include <vector>
#include <boost/thread/thread.hpp>
#include <random>
#include <string>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/SVD>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

#include "CommonUtils.h"
using namespace std;
using namespace Eigen;


void RunViewer() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    vector<Vector3f> points;
    points = ReadPointCloudFromFile("/root/workspace/res/king.xyz");
    cloud = ToPCLVecArray(points);
    auto obbBox = OBB(points);
    vector<Eigen::Vector3f> boxPoint;
    vector<std::pair<int, int>> indeies;
    GetBoxLines(obbBox, boxPoint, indeies);

    cloud2 = cloud;
    AddNoiseToPointCloud( cloud,cloud2);
    vector<Vector3f> points2 = ToEigenVecArray(cloud2);
    auto obbBox2 = OBB(points2);
    vector<Eigen::Vector3f> boxPoint2;
    vector<std::pair<int, int>> indeies2;
    GetBoxLines(obbBox2, boxPoint2, indeies2);

    // 可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Points"));
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pCloudPtr(&cloud);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pCloudPtr2(&cloud2);
    viewer->addPointCloud<pcl::PointXYZ>(pCloudPtr, "cloud");
    viewer->addPointCloud<pcl::PointXYZ>(pCloudPtr2, "cloud2");

    for (int i = 0; i < indeies.size(); i++) {
        pcl::PointXYZ l1, l2;
        l1 = ToPclVec(boxPoint[indeies[i].first]);
        l2 = ToPclVec(boxPoint[indeies[i].second]);
        viewer->addLine(l1, l2, "line_1_" + std::to_string(i));
    }
    for (int i = 0; i < indeies2.size(); i++) {
        pcl::PointXYZ l1, l2;
        l1 = ToPclVec(boxPoint2[indeies2[i].first]);
        l2 = ToPclVec(boxPoint2[indeies2[i].second]);
        viewer->addLine(l1, l2, "line_2_" + std::to_string(i));
    }
    while (!viewer->wasStopped()) {
        viewer->spin();
    }
}
int main (int argc, char** argv)
{
    RunViewer();
    return (0);
}


