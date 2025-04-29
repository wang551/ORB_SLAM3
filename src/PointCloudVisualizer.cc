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

#include "PointCloudVisualizer.h"
#include <iostream>
#include <fstream>
#include <iomanip>

namespace ORB_SLAM3
{

PointCloudVisualizer::PointCloudVisualizer(Atlas* pAtlas) : 
    mpAtlas(pAtlas), mpCurrentMap(nullptr), mbUseColor(true)
{
    if(mpAtlas)
        mpCurrentMap = mpAtlas->GetCurrentMap();
}

PointCloudVisualizer::~PointCloudVisualizer()
{
}

void PointCloudVisualizer::SetAtlas(Atlas* pAtlas)
{
    mpAtlas = pAtlas;
    if(mpAtlas)
        mpCurrentMap = mpAtlas->GetCurrentMap();
}

void PointCloudVisualizer::GeneratePointCloud()
{
    if(!mpAtlas)
        return;
        
    unique_lock<mutex> lock(mMutexPointCloud);
    
    // 清空之前的点云数据
    mvPointCloudPos.clear();
    mvPointCloudColor.clear();

    // 从Atlas中获取所有地图
    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    std::cout << "Atlas 中地图数量: " << vpMaps.size() << std::endl;

    // 预计算总地图点数量
    int totalMapPoints = 0;
    for (Map* pMap : vpMaps) {
        if (pMap) {
            vector<MapPoint*> vpMapPoints = pMap->GetAllMapPoints();
            totalMapPoints += vpMapPoints.size();
        }
    }

    // 预分配内存以提高效率
    mvPointCloudPos.reserve(totalMapPoints);

    if(mbUseColor)
        mvPointCloudColor.reserve(totalMapPoints);

    // 处理所有地图中的所有地图点
    for (Map* pMap : vpMaps) {
        if (!pMap)
            continue;

        std::cout << "处理地图 ID " << pMap->GetId() << std::endl;
        vector<MapPoint*> vpMapPoints = pMap->GetAllMapPoints();
        std::cout << "地图点数量: " << vpMapPoints.size() << std::endl;

        for (MapPoint* pMP : vpMapPoints) {
            // 跳过无效的地图点
            if(!pMP || pMP->isBad())
                continue;
                
            // 获取地图点的世界坐标
            Eigen::Vector3f pos = pMP->GetWorldPos();
            mvPointCloudPos.push_back(pos);
            
            // 如果启用颜色，则尝试获取地图点的颜色
            if(mbUseColor)
            {
                // 简单的根据位置生成颜色（可以替换为实际颜色或者通过其他特征计算）
                // 在实际应用中，可以使用从图像中提取的颜色
                float r = (pos(0) + 10) / 20.0f; // 将x坐标归一化到[0,1]
                float g = (pos(1) + 10) / 20.0f; // 将y坐标归一化到[0,1]
                float b = (pos(2) + 10) / 20.0f; // 将z坐标归一化到[0,1]
                
                // 确保RGB在[0,1]范围内
                r = std::min(std::max(r, 0.0f), 1.0f);
                g = std::min(std::max(g, 0.0f), 1.0f);
                b = std::min(std::max(b, 0.0f), 1.0f);
                
                mvPointCloudColor.push_back(Eigen::Vector3f(r, g, b));
            }
        }
    }
    
    // 生成关键帧轨迹 - 从所有地图中获取所有关键帧
    vector<KeyFrame*> vpAllKFs;
    for (Map* pMap : vpMaps) {
        if (!pMap)
            continue;
            
        const vector<KeyFrame*> &vpKFs = pMap->GetAllKeyFrames();
        vpAllKFs.insert(vpAllKFs.end(), vpKFs.begin(), vpKFs.end());
    }
    
    // 清空之前的轨迹数据
    mvKeyFrameTrajectoryPos.clear();
    mvKeyFrameTrajectoryColor.clear();
    
    // 预分配内存
    mvKeyFrameTrajectoryPos.reserve(vpAllKFs.size());
    mvKeyFrameTrajectoryColor.reserve(vpAllKFs.size());
    
    for(size_t i=0; i<vpAllKFs.size(); i++)
    {
        KeyFrame* pKF = vpAllKFs[i];
        
        if(!pKF || pKF->isBad())
            continue;
            
        // 获取关键帧的位置（相机中心）
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Eigen::Vector3f twc = Twc.translation();
        
        mvKeyFrameTrajectoryPos.push_back(twc);
        
        // 为关键帧轨迹设置固定颜色（蓝色）
        mvKeyFrameTrajectoryColor.push_back(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    }
    
    std::cout << "生成点云完成: " << mvPointCloudPos.size() << " 个点, " 
              << mvKeyFrameTrajectoryPos.size() << " 个关键帧轨迹点" << std::endl;
}

bool PointCloudVisualizer::SavePointCloudToPLY(const std::string &filename)
{
    unique_lock<mutex> lock(mMutexPointCloud);
    
    // 如果点云为空，先生成点云
    if(mvPointCloudPos.empty())
        GeneratePointCloud();
        
    if(mvPointCloudPos.empty())
        return false;
    
    // 打开文件
    std::ofstream file(filename.c_str());
    if(!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }
    
    // 写入PLY文件头
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << mvPointCloudPos.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    
    // 如果有颜色数据，添加颜色属性
    if(mbUseColor && mvPointCloudColor.size() == mvPointCloudPos.size())
    {
        file << "property uchar red" << std::endl;
        file << "property uchar green" << std::endl;
        file << "property uchar blue" << std::endl;
    }
    
    file << "end_header" << std::endl;
    
    // 写入点云数据
    for(size_t i=0; i<mvPointCloudPos.size(); i++)
    {
        const Eigen::Vector3f &pos = mvPointCloudPos[i];
        
        file << pos(0) << " " << pos(1) << " " << pos(2);
        
        // 如果有颜色，写入颜色数据
        if(mbUseColor && i < mvPointCloudColor.size())
        {
            const Eigen::Vector3f &color = mvPointCloudColor[i];
            int r = static_cast<int>(color(0) * 255);
            int g = static_cast<int>(color(1) * 255);
            int b = static_cast<int>(color(2) * 255);
            
            file << " " << r << " " << g << " " << b;
        }
        
        file << std::endl;
    }
    
    file.close();
    
    std::cout << "点云已保存到: " << filename << std::endl;
    return true;
}

bool PointCloudVisualizer::SavePointCloudToPCD(const std::string &filename)
{
    unique_lock<mutex> lock(mMutexPointCloud);
    
    // 如果点云为空，先生成点云
    if(mvPointCloudPos.empty())
        GeneratePointCloud();
        
    if(mvPointCloudPos.empty())
        return false;
    
    // 打开文件
    std::ofstream file(filename.c_str());
    if(!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }
    
    // 写入PCD文件头
    file << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    file << "VERSION .7" << std::endl;
    file << "FIELDS x y z";
    if(mbUseColor && mvPointCloudColor.size() == mvPointCloudPos.size())
        file << " rgb";
    file << std::endl;
    
    file << "SIZE 4 4 4";
    if(mbUseColor && mvPointCloudColor.size() == mvPointCloudPos.size())
        file << " 4";
    file << std::endl;
    
    file << "TYPE F F F";
    if(mbUseColor && mvPointCloudColor.size() == mvPointCloudPos.size())
        file << " U";
    file << std::endl;
    
    file << "COUNT 1 1 1";
    if(mbUseColor && mvPointCloudColor.size() == mvPointCloudPos.size())
        file << " 1";
    file << std::endl;
    
    file << "WIDTH " << mvPointCloudPos.size() << std::endl;
    file << "HEIGHT 1" << std::endl;
    file << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    file << "POINTS " << mvPointCloudPos.size() << std::endl;
    file << "DATA ascii" << std::endl;
    
    // 写入点云数据
    for(size_t i=0; i<mvPointCloudPos.size(); i++)
    {
        const Eigen::Vector3f &pos = mvPointCloudPos[i];
        
        file << pos(0) << " " << pos(1) << " " << pos(2);
        
        // 如果有颜色，写入颜色数据
        if(mbUseColor && i < mvPointCloudColor.size())
        {
            const Eigen::Vector3f &color = mvPointCloudColor[i];
            unsigned int rgb_packed = 
                static_cast<unsigned int>(color(0) * 255) << 16 | 
                static_cast<unsigned int>(color(1) * 255) << 8 | 
                static_cast<unsigned int>(color(2) * 255);
            
            file << " " << rgb_packed;
        }
        
        file << std::endl;
    }
    
    file.close();
    
    std::cout << "点云已保存到: " << filename << std::endl;
    return true;
}

bool PointCloudVisualizer::SaveKeyFrameTrajectoryToPLY(const std::string &filename)
{
    unique_lock<mutex> lock(mMutexPointCloud);
    
    // 如果轨迹为空，先生成点云
    if(mvKeyFrameTrajectoryPos.empty())
        GeneratePointCloud();
        
    if(mvKeyFrameTrajectoryPos.empty())
        return false;
    
    // 打开文件
    std::ofstream file(filename.c_str());
    if(!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }
    
    // 写入PLY文件头
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << mvKeyFrameTrajectoryPos.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "end_header" << std::endl;
    
    // 写入轨迹点数据
    for(size_t i=0; i<mvKeyFrameTrajectoryPos.size(); i++)
    {
        const Eigen::Vector3f &pos = mvKeyFrameTrajectoryPos[i];
        
        file << pos(0) << " " << pos(1) << " " << pos(2);
        
        // 为轨迹设置颜色
        if(i < mvKeyFrameTrajectoryColor.size())
        {
            const Eigen::Vector3f &color = mvKeyFrameTrajectoryColor[i];
            int r = static_cast<int>(color(0) * 255);
            int g = static_cast<int>(color(1) * 255);
            int b = static_cast<int>(color(2) * 255);
            
            file << " " << r << " " << g << " " << b;
        }
        else
        {
            // 默认蓝色
            file << " 0 0 255";
        }
        
        file << std::endl;
    }
    
    file.close();
    
    std::cout << "关键帧轨迹已保存到: " << filename << std::endl;
    return true;
}

void PointCloudVisualizer::DrawPointCloud()
{
    unique_lock<mutex> lock(mMutexPointCloud);
    
    // 绘制地图点
    glPointSize(2);
    glBegin(GL_POINTS);
    
    for(size_t i=0; i<mvPointCloudPos.size(); i++)
    {
        const Eigen::Vector3f &pos = mvPointCloudPos[i];
        
        // 设置颜色（如果可用）
        if(mbUseColor && i < mvPointCloudColor.size())
        {
            const Eigen::Vector3f &color = mvPointCloudColor[i];
            glColor3f(color(0), color(1), color(2));
        }
        else
        {
            // 默认白色
            glColor3f(1.0, 1.0, 1.0);
        }
        
        glVertex3f(pos(0), pos(1), pos(2));
    }
    
    glEnd();
    
    // 绘制关键帧轨迹
    glLineWidth(2);
    glBegin(GL_LINE_STRIP);
    
    // 轨迹使用蓝色
    glColor3f(0.0, 0.0, 1.0);
    
    for(size_t i=0; i<mvKeyFrameTrajectoryPos.size(); i++)
    {
        const Eigen::Vector3f &pos = mvKeyFrameTrajectoryPos[i];
        glVertex3f(pos(0), pos(1), pos(2));
    }
    
    glEnd();
}

bool PointCloudVisualizer::LoadPointCloudFromPLY(const std::string &filename)
{
    unique_lock<mutex> lock(mMutexPointCloud);
    
    // 打开文件
    std::ifstream file(filename.c_str());
    if(!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }
    
    // 读取PLY文件头
    std::string line;
    bool header_end = false;
    bool has_color = false;
    int num_vertices = 0;
    
    while(!header_end && std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;
        
        if(keyword == "element")
        {
            std::string elem_type;
            iss >> elem_type;
            
            if(elem_type == "vertex")
                iss >> num_vertices;
        }
        else if(keyword == "property")
        {
            std::string data_type, prop_name;
            iss >> data_type >> prop_name;
            
            if(prop_name == "red" || prop_name == "green" || prop_name == "blue")
                has_color = true;
        }
        else if(keyword == "end_header")
        {
            header_end = true;
        }
    }
    
    if(!header_end)
    {
        std::cerr << "无效的PLY文件格式" << std::endl;
        return false;
    }
    
    // 清空当前数据
    mvPointCloudPos.clear();
    mvPointCloudColor.clear();
    
    // 预分配内存
    mvPointCloudPos.reserve(num_vertices);
    if(has_color)
        mvPointCloudColor.reserve(num_vertices);
    
    // 读取顶点数据
    for(int i = 0; i < num_vertices && std::getline(file, line); i++)
    {
        std::istringstream iss(line);
        float x, y, z;
        
        if(!(iss >> x >> y >> z))
            continue;
        
        mvPointCloudPos.push_back(Eigen::Vector3f(x, y, z));
        
        if(has_color)
        {
            int r, g, b;
            if(iss >> r >> g >> b)
            {
                float rf = r / 255.0f;
                float gf = g / 255.0f;
                float bf = b / 255.0f;
                mvPointCloudColor.push_back(Eigen::Vector3f(rf, gf, bf));
            }
        }
    }
    
    file.close();
    
    if(has_color)
        mbUseColor = true;
    
    std::cout << "从文件加载点云: " << filename << std::endl;
    std::cout << "点数: " << mvPointCloudPos.size() << std::endl;
    std::cout << "带颜色: " << (has_color ? "是" : "否") << std::endl;
    
    return true;
}

} // namespace ORB_SLAM3