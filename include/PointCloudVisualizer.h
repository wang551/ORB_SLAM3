/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef POINTCLOUDVISUALIZER_H
#define POINTCLOUDVISUALIZER_H

#include <pangolin/pangolin.h>
#include <mutex>
#include <string>
#include <fstream>

#include "Map.h"
#include "Atlas.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "System.h"

namespace ORB_SLAM3
{

class PointCloudVisualizer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // 构造函数
    PointCloudVisualizer(Atlas* pAtlas);
    
    // 析构函数
    ~PointCloudVisualizer();
    
    // 设置Atlas
    void SetAtlas(Atlas* pAtlas);
    
    // 从当前地图生成点云
    void GeneratePointCloud();
    
    // 保存点云到PLY文件，可以被PCL等工具加载
    bool SavePointCloudToPLY(const std::string &filename);
    
    // 保存点云到PCD文件，PCL标准格式
    bool SavePointCloudToPCD(const std::string &filename);
    
    // 保存关键帧轨迹点云
    bool SaveKeyFrameTrajectoryToPLY(const std::string &filename);
    
    // 可视化点云
    void DrawPointCloud();

    // 从文件加载点云
    bool LoadPointCloudFromPLY(const std::string &filename);
    
    // 获取点云位置数据（用于外部处理，如Read.cc）
    const std::vector<Eigen::Vector3f>& GetPointCloudPositions() const { return mvPointCloudPos; }
    
    // 获取点云颜色数据
    const std::vector<Eigen::Vector3f>& GetPointCloudColors() const { return mvPointCloudColor; }
    
    // 获取关键帧轨迹点位置数据
    const std::vector<Eigen::Vector3f>& GetKeyFrameTrajectoryPositions() const { return mvKeyFrameTrajectoryPos; }
    
    // 获取关键帧轨迹点颜色数据
    const std::vector<Eigen::Vector3f>& GetKeyFrameTrajectoryColors() const { return mvKeyFrameTrajectoryColor; }
    
private:
    // 指向Atlas的指针
    Atlas* mpAtlas;
    
    // 当前地图
    Map* mpCurrentMap;
    
    // 访问地图的互斥锁
    std::mutex mMutexPointCloud;
    
    // 点云数据：位置
    std::vector<Eigen::Vector3f> mvPointCloudPos;
    
    // 点云数据：颜色 (如果可用)
    std::vector<Eigen::Vector3f> mvPointCloudColor;
    
    // 是否使用颜色
    bool mbUseColor;
    
    // 帧轨迹点云数据
    std::vector<Eigen::Vector3f> mvKeyFrameTrajectoryPos;
    std::vector<Eigen::Vector3f> mvKeyFrameTrajectoryColor;
};

}// namespace ORB_SLAM

#endif // POINTCLOUDVISUALIZER_H