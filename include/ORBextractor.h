/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:
	//建立图像金字塔
	//将原始图像一级级缩小并依次存在mvImagePyramid里
    void ComputePyramid(cv::Mat image); 
	//利用四叉树提取高斯金字塔中每层图像的orb关键点   
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints); 
    //将关键点分配到四叉树，筛选关键点
	std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);
	//作者遗留下的旧的orb关键点方法
    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
	//存储关键点附近patch的点对相对位置
    std::vector<cv::Point> pattern;
	
	//这几个变量就是初始化的时候读取的摄像头的yaml配置文件得到的
    int nfeatures;
    double scaleFactor;//每层之间的缩放比例，kitti里面是1.2
    int nlevels;
    int iniThFAST; //提取FAST角点时初始阈值
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;//每层的特征数量

    std::vector<int> umax;//描述Patch圆直径为PATCH_SIZE时一圈离散点的坐标。下标代表v方向的坐标，下标对应的值代表u方向的坐标

    std::vector<float> mvScaleFactor;//每层的相对于原始图像的缩放比例,第0个为1，第二个为1.2（kitti）
    std::vector<float> mvInvScaleFactor;//mvScaleFactor的倒数    
    std::vector<float> mvLevelSigma2;//mvScaleFactor的平方
    std::vector<float> mvInvLevelSigma2;//mvScaleFactor的平方的倒数
};

} //namespace ORB_SLAM

#endif

