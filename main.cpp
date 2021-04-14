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
#include "ICP.h"
using namespace std;
using namespace Eigen;

void RunViewer() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    pcl::PointCloud<pcl::PointXYZRGBA> cloud3;
    vector<Vector3f> points;
    points = ReadPointCloudFromFile("/root/workspace/res/king.xyz");
    cloud = ToPCLVecArray(points);
    auto obbBox = OBB(points);
    DrawableBox drawableBox = GetDrawableBox(obbBox);

    cloud2 = cloud;
    AddNoiseToPointCloud(cloud, cloud2);
    vector<Vector3f> points2 = ToEigenVecArray(cloud2);
    auto obbBox2 = OBB(points2);
    DrawableBox drawableBox2 = GetDrawableBox(obbBox2);

    vector<Vector3f> points3;
    ICP_Result trans = ICP_Rotation(points,points2);
    ApplyTrans(points,trans,points3);
    cloud3 =  ToPCLColorVecArray(points3,0xff00ffff);

    // 可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Points"));
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pCloudPtr(&cloud);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pCloudPtr2(&cloud2);
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pCloudPtr3(&cloud3);
    viewer->addPointCloud<pcl::PointXYZ>(pCloudPtr, "cloud");
    viewer->addPointCloud<pcl::PointXYZ>(pCloudPtr2, "cloud2");
    viewer->addPointCloud<pcl::PointXYZRGBA>(pCloudPtr3, "cloud3");
    drawableBox.Draw(viewer,  "box1_", 0xffff00ff);
    drawableBox2.Draw(viewer, "box2_", 0xff00ffff);

    while (!viewer->wasStopped()) {
        viewer->spin();
    }
}
int main (int argc, char** argv)
{
    RunViewer();
    return (0);
}


