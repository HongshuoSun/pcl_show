//
// Created by bugma on 2021/4/14.
//

#ifndef PCL_SHOW_ICP_H
#define PCL_SHOW_ICP_H
#include "CommonUtils.h"
struct ICP_Result{
    Matrix3f rot;
    Vector3f translation;
};
ICP_Result ICP_Rotation(const std::vector<Eigen::Vector3f>& src,
                  const std::vector<Eigen::Vector3f>& dst ){
    ICP_Result ans;
    Vector3f srcMean(0.,0.,0.),dstMean(0.,0.,0.);
    int pointCount = src.size();
    assert(dst.size()==pointCount);
    for(int i=0;i<pointCount;i++){
        srcMean+=src[i];
        dstMean+=dst[i];
    }
    srcMean*=(1./static_cast<float>(pointCount));
    dstMean*=(1./static_cast<float>(pointCount));
    Matrix3f rot;
    rot.setZero();
    for(int i=0;i<pointCount;i++){
        rot+= (src[i]-srcMean)*((dst[i]-dstMean).transpose());
    }
    JacobiSVD<Matrix3f> svd(rot, ComputeFullV| ComputeFullU);
    rot =  svd.matrixV()* (svd.matrixU().transpose());
    printf("rot det = %f\n",rot.determinant());
    Vector3f trans(0.,0.,0.);
    trans = dstMean-rot*srcMean;
    ans.rot = rot;
    ans.translation = trans;
    return ans;
}
void ApplyTrans(const std::vector<Eigen::Vector3f>& src,const ICP_Result& trans,
                         std::vector<Eigen::Vector3f>& dst ){
    int pointCount = src.size();
    dst.resize(pointCount);
    for(int i=0;i<pointCount;i++){
        dst[i] = trans.rot*src[i]+trans.translation;
    }
}


#endif //PCL_SHOW_ICP_H
