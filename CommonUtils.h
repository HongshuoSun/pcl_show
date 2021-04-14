//
// Created by bugma on 2021/4/12.
//

#ifndef PCL_SHOW_COMMONUTILS_H
#define PCL_SHOW_COMMONUTILS_H
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
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>


using namespace  std;
using namespace  Eigen;
struct OBB_BOX{
    Matrix3f rot;
    Vector3f center;
    Vector3f minVertex;
    Vector3f maxVertex;
};
pcl::PointXYZRGBA ToPclColorVec(const Eigen::Vector3f& v ,const uint32_t rgba);
struct DrawableBox{
    vector<Eigen::Vector3f> boxPoint;
    vector<std::pair<int, int>> indeies;
    void Draw(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,const std::string& prefixName,
              uint32_t rgba = 0xffffffff){
        for (int i = 0; i < indeies.size(); i++) {
            pcl::PointXYZRGBA l1, l2;
            l1 = ToPclColorVec(boxPoint[indeies[i].first],rgba);
            l2 = ToPclColorVec(boxPoint[indeies[i].second],rgba);
            viewer->addLine(l1, l2,prefixName + std::to_string(i));
        }
    }
};

std::vector<Eigen::Vector3f> ReadPointCloudFromFile(const char* filePath){
    assert(filePath!= nullptr);
    std::vector<Eigen::Vector3f> pc;
    std::ifstream fs(filePath);
    if(!fs.is_open()){
        assert(false);
        return pc;
    }
    float x=0,y=0,z=0;
    while(fs>>x>>y>>z){
        pc.push_back(Eigen::Vector3f(x,y,z));
    }
    printf("read %d points.\n",static_cast<int>(pc.size()));
    return pc;
}
void GetWidthAndHeight(int totalCount,int& width,int& height){
    if(totalCount<100){
        width = totalCount;
        height = 1;
        return;
    }
    int sqrtCount = std::sqrt(totalCount)+1;
    while( (totalCount%sqrtCount) != 0){
        sqrtCount--;
    }
    width = sqrtCount;
    height = totalCount /width;
    assert((width*height)==totalCount);
}

void GetBoxLines(const OBB_BOX& box,vector<Eigen::Vector3f>& boxPoint,vector<std::pair<int,int>>& indeies) {

    boxPoint.resize(8);
    indeies.resize(12);

    boxPoint[0] = Eigen::Vector3f(box.minVertex.x(), box.minVertex.y(), box.minVertex.z());
    boxPoint[1] = Eigen::Vector3f(box.minVertex.x(), box.maxVertex.y(), box.minVertex.z());
    boxPoint[2] = Eigen::Vector3f(box.maxVertex.x(), box.maxVertex.y(), box.minVertex.z());
    boxPoint[3] = Eigen::Vector3f(box.maxVertex.x(), box.minVertex.y(), box.minVertex.z());

    boxPoint[4] = Eigen::Vector3f(box.minVertex.x(), box.minVertex.y(), box.maxVertex.z());
    boxPoint[5] = Eigen::Vector3f(box.minVertex.x(), box.maxVertex.y(), box.maxVertex.z());
    boxPoint[6] = Eigen::Vector3f(box.maxVertex.x(), box.maxVertex.y(), box.maxVertex.z());
    boxPoint[7] = Eigen::Vector3f(box.maxVertex.x(), box.minVertex.y(), box.maxVertex.z());


    for (int i = 0; i < boxPoint.size(); i++) {
        boxPoint[i]=box.rot.transpose()*boxPoint[i];
        boxPoint[i]+=box.center;
    }

    indeies[0] = std::pair<int, int>(0, 1);
    indeies[1] = std::pair<int, int>(1, 2);
    indeies[2] = std::pair<int, int>(2, 3);
    indeies[3] = std::pair<int, int>(3, 0);
    indeies[4] = std::pair<int, int>(4, 5);
    indeies[5] = std::pair<int, int>(5, 6);
    indeies[6] = std::pair<int, int>(6, 7);
    indeies[7] = std::pair<int, int>(7, 4);

    indeies[8] = std::pair<int, int>(0, 4);
    indeies[9] = std::pair<int, int>(1, 5);
    indeies[10] = std::pair<int, int>(2, 6);
    indeies[11] = std::pair<int, int>(3, 7);

}
DrawableBox GetDrawableBox(const OBB_BOX& box){
    DrawableBox drawableBox;
    GetBoxLines(box,drawableBox.boxPoint,drawableBox.indeies);
    return drawableBox;
}
OBB_BOX OBB(vector<Vector3f> points) {
    OBB_BOX obbBox;
    vector<Vector3f> ans;
    int pointSize = points.size();
    float pointSizeF = static_cast<float>(pointSize);
    float sx, sy, sz;
    sx = sy = sz = 0.;
    for (const auto &p :points) {
        sx += p.x();
        sy += p.y();
        sz += p.z();
    }
    sx /= pointSizeF;
    sy /= pointSizeF;
    sz /= pointSizeF;
    Vector3f center(sx,sy,sz);
    float covXX, covXY, covXZ, covYZ, covYY, covZZ;
    covXZ = covXX = covXY = covYZ = covYY = covZZ = 0.;
    for (const auto &p :points) {
        float x = p.x() - sx;
        float y = p.y() - sy;
        float z = p.z() - sz;
        covXX += x * x;
        covYY += y * y;
        covZZ += z * z;
        covXY += x * y;
        covXZ += x * z;
        covYZ += y * z;
    }
    covXX /= pointSizeF;
    covYY /= pointSizeF;
    covZZ /= pointSizeF;
    covXY /= pointSizeF;
    covXZ /= pointSizeF;
    covYZ /= pointSizeF;
    // Matrix3d m3 {{1.2,2.2,3.3},{4.2,2.5,6.3},{7.2,8.2,9.3}};
    Eigen::Matrix3f covMat;
    covMat << covXX, covXY, covXZ, covXY, covYY, covYZ, covXZ, covYZ, covZZ;
    std::cout << covMat << std::endl;
    SelfAdjointEigenSolver<Eigen::Matrix3f> covEng(covMat);

    auto eigenVectors = covEng.eigenvectors();
    Matrix3f eigVec;
    Vector3f eigVal= covEng.eigenvalues();
    eigVal.reverseInPlace();
    for(int i=0;i<3;i++){
        eigVec.col(i) = eigenVectors.col(3-i-1);
    }
    auto diag = Eigen::DiagonalMatrix<float,3>(eigVal);
    auto eigVecTrans = eigVec.transpose();
    std::cout<<"det:"<<eigVec.determinant()<<endl;
    std::cout<<"eigVec:\t\n"<<eigVec<<endl;
    std::cout<<"eigVal:\t\n"<<eigVal<<endl;
    std::cout<<"error:\n"<<(covMat - eigVec*diag*eigVec.transpose()).norm()<<endl;
    float maxX,minX,maxY,minY,maxZ,minZ;
    {
        Vector3f point0Rot = eigVecTrans*(points[0]-center);
        maxX=minX = point0Rot.x();
        maxY=minY = point0Rot.y();
        maxZ=minZ = point0Rot.z();
    }
    for(int i=1;i<pointSize;i++){
        Vector3f pointRot = eigVecTrans*(points[i]-center);
        minX = std::min(minX,pointRot.x());
        maxX = std::max(maxX,pointRot.x());
        minY = std::min(minY,pointRot.y());
        maxY = std::max(maxY,pointRot.y());
        minZ = std::min(minZ,pointRot.z());
        maxZ = std::max(maxZ,pointRot.z());
    }
    obbBox.minVertex=Vector3f(minX,minY,minZ);
    obbBox.maxVertex=Vector3f(maxX,maxY,maxZ);
    obbBox.center = center;
    obbBox.rot = eigVecTrans;
    return obbBox;

}
vector<Vector3f> GenRandomPoints(int pointSize,
                                 std::pair<float,float> xRange=std::pair<float,float>(-0.5f,0.5f),
                                 std::pair<float,float> yRange=std::pair<float,float>(-0.5f,0.5f),
                                 std::pair<float,float> zRange=std::pair<float,float>(-0.5f,0.5f)) {
    vector<Vector3f> ans;
    ans.resize(pointSize);
    std::default_random_engine generator;
    std::normal_distribution<double> xDis(0,1);
    std::normal_distribution<double> zDis(0,2);
    std::normal_distribution<double> yDis(0,3);
    for(int i=0;i<pointSize;i++) {
        float x, y, z;
        x = xDis(generator);
        y = yDis(generator);
        z = zDis(generator);
        ans[i] = Vector3f{x, y, z};
    }
    return ans;
}
void SavePointCloudToFile( const pcl::PointCloud<pcl::PointXYZRGB>& cloud,const string& fileName){
    pcl::io::savePCDFileASCII (fileName.c_str(), cloud);
}
Eigen::Vector3f ToEigenVec(const pcl::PointXYZ& p){
    return Eigen::Vector3f(p.x,p.y,p.z);
}
pcl::PointXYZ ToPclVec(const Eigen::Vector3f& v ){
    return pcl::PointXYZ(v.x(),v.y(),v.z());
}
pcl::PointXYZRGBA ToPclColorVec(const Eigen::Vector3f& v ,const uint32_t rgba){
     pcl::PointXYZRGBA pclPoint;
     pclPoint.x = v.x();
     pclPoint.y = v.y();
     pclPoint.z = v.z();
     pclPoint.rgba = rgba;
     return pclPoint;
}
void AddNoiseToPointCloud( const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& dest){
    int pointSize = cloud.size();
    dest.resize(pointSize);
    Eigen::Vector3f center(0.,0.,0.);
    for(int i=0;i<pointSize;i++){
        center+=Eigen::Vector3f(cloud[i].x,cloud[i].y,cloud[i].z);
    }
    center *= (1./static_cast<float>(pointSize));
    Vector3f newCenter(100,20,0);


    std::default_random_engine generator;
    std::normal_distribution<double> xDist(0,2);
    std::normal_distribution<double> yDist(0,2);
    std::normal_distribution<double> zDist(0,4);
    AngleAxis<float> randRot(static_cast<float>((drand48()-0.5f)*2.f*M_PI), Vector3f(drand48(),drand48(),drand48()).normalized());

    for(int i=0;i<pointSize;i++){
        Eigen::Vector3f normalizerPoint = ToEigenVec(cloud[i])-center;
        normalizerPoint+=Vector3f(xDist(generator), yDist(generator),zDist(generator));
        normalizerPoint = randRot*normalizerPoint;
        normalizerPoint+=newCenter;
        dest[i] = ToPclVec(normalizerPoint);
    }
}
std::vector<Eigen::Vector3f> ToEigenVecArray(const  pcl::PointCloud<pcl::PointXYZ>& cloudPoint){
    int size = cloudPoint.size();
    std::vector<Eigen::Vector3f> ans;
    ans.resize(size);
    for(int i=0;i<size;i++){
        ans[i] = ToEigenVec(cloudPoint.points[i]);
    }
    return ans;
}
pcl::PointCloud<pcl::PointXYZ> ToPCLVecArray(const std::vector<Eigen::Vector3f>& eigVec){
    int size = eigVec.size();
    pcl::PointCloud<pcl::PointXYZ> ans;
    int width = 0;
    int height = 0;
    GetWidthAndHeight(size, width, height);
    ans.width = width;
    ans.height = height;
    ans.is_dense = true;
    ans.points.resize(size);
    for(int i=0;i<size;i++){
        ans[i] = ToPclVec(eigVec[i]);
    }
    return ans;
}
pcl::PointCloud<pcl::PointXYZRGBA> ToPCLColorVecArray(const std::vector<Eigen::Vector3f>& eigVec,uint32_t rgba){
    int size = eigVec.size();
    pcl::PointCloud<pcl::PointXYZRGBA> ans;
    int width = 0;
    int height = 0;
    GetWidthAndHeight(size, width, height);
    ans.width = width;
    ans.height = height;
    ans.is_dense = true;
    ans.points.resize(size);
    for(int i=0;i<size;i++){
        ans[i] = ToPclColorVec(eigVec[i],rgba);
    }
    return ans;
}


#endif //PCL_SHOW_COMMONUTILS_H
