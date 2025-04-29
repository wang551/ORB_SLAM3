/**
 * 这个程序用于读取ORB_SLAM3保存的.osa文件，并可视化其中的点云数据
 */

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<string>
#include<thread>

// Boost 序列化库头文件
#include<boost/archive/binary_iarchive.hpp>
#include<boost/archive/text_iarchive.hpp>
#include<boost/archive/binary_oarchive.hpp>
#include<boost/archive/text_oarchive.hpp>
#include<boost/serialization/string.hpp>
#include<boost/serialization/vector.hpp>

// ORB-SLAM3
#include "System.h"
#include "PointCloudVisualizer.h"

// PCL可视化库
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/voxel_grid.h>

using namespace std;

void ShowHelpText()
{
    cout << "使用方法: ./Read path_to_file.osa path_to_settings.yaml [options]" << endl;
    cout << "选项:" << endl;
    cout << "  --save-pcd filename.pcd   将点云保存为PCD文件" << endl;
    cout << "  --save-ply filename.ply   将点云保存为PLY文件" << endl;
    cout << "  --save-trajectory filename.ply  将关键帧轨迹保存为PLY文件" << endl;
    cout << "  --resolution value        设置下采样分辨率 (默认: 0.01)" << endl;
    cout << "  --vocabulary path         指定ORB词汇表路径 (默认: ../../../Vocabulary/ORBvoc.txt)" << endl;
}

// 将ORB_SLAM3的点云转换为PCL点云
bool ConvertToPCLPointCloud(ORB_SLAM3::PointCloudVisualizer* pVisualizer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud)
{
    // 生成点云数据
    pVisualizer->GeneratePointCloud();
    
    // 获取点云数据的个数
    const auto& positions = pVisualizer->GetPointCloudPositions();
    const auto& colors = pVisualizer->GetPointCloudColors();
    
    if(positions.empty())
    {
        cerr << "\033[1;31m错误: 未能从文件中提取点云数据\033[0m" << endl;
        return false;
    }
    
    pointCloud->clear();
    pointCloud->reserve(positions.size());
    
    cout << "正在转换 " << positions.size() << " 个点..." << endl;
    
    // 将ORB_SLAM3的点云转换为PCL点云
    for(size_t i = 0; i < positions.size(); ++i)
    {
        pcl::PointXYZRGB point;
        point.x = positions[i][0];
        point.y = positions[i][1];
        point.z = positions[i][2];
        
        // 如果有颜色信息，就使用它
        if(i < colors.size())
        {
            point.r = static_cast<uint8_t>(colors[i][0] * 255);
            point.g = static_cast<uint8_t>(colors[i][1] * 255);
            point.b = static_cast<uint8_t>(colors[i][2] * 255);
        }
        else
        {
            // 根据坐标位置设置不同颜色
            if (point.x > 0 && point.y > 0) {
                point.r = 255; point.g = 0; point.b = 0;  // 红色
            } else if (point.x < 0 && point.y > 0) {
                point.r = 0; point.g = 255; point.b = 0;  // 绿色
            } else if (point.x > 0 && point.y < 0) {
                point.r = 0; point.g = 0; point.b = 255;  // 蓝色
            } else {
                point.r = 255; point.g = 255; point.b = 0;  // 黄色
            }
        }
        
        pointCloud->points.push_back(point);
    }
    
    pointCloud->width = pointCloud->points.size();
    pointCloud->height = 1;
    pointCloud->is_dense = false;
    
    cout << "成功转换了 " << pointCloud->points.size() << " 个点" << endl;
    return true;
}

// 添加轨迹点云到PCL可视化器
void AddTrajectoryToVisualizer(ORB_SLAM3::PointCloudVisualizer* pVisualizer, 
                              pcl::visualization::PCLVisualizer::Ptr viewer)
{
    const auto& positions = pVisualizer->GetKeyFrameTrajectoryPositions();
    
    if(positions.empty())
        return;
    
    cout << "正在添加 " << positions.size() << " 个轨迹点..." << endl;
    
    // 为轨迹创建PCL点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trajectoryCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    trajectoryCloud->reserve(positions.size());
    
    // 添加轨迹点
    for(size_t i = 0; i < positions.size(); ++i)
    {
        pcl::PointXYZRGB point;
        point.x = positions[i][0];
        point.y = positions[i][1];
        point.z = positions[i][2];
        
        // 轨迹点使用蓝色
        point.r = 0; 
        point.g = 0; 
        point.b = 255;
        
        trajectoryCloud->points.push_back(point);
    }
    
    trajectoryCloud->width = trajectoryCloud->points.size();
    trajectoryCloud->height = 1;
    trajectoryCloud->is_dense = false;
    
    // 添加轨迹点云到可视化器
    viewer->addPointCloud<pcl::PointXYZRGB>(trajectoryCloud, "trajectory cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "trajectory cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "trajectory cloud");
    
    // 添加轨迹线
    for(size_t i = 1; i < positions.size(); ++i)
    {
        string line_id = "line_" + to_string(i);
        viewer->addLine<pcl::PointXYZRGB>(trajectoryCloud->points[i-1], trajectoryCloud->points[i], 0, 0, 1, line_id);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, line_id);
    }
}

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        ShowHelpText();
        return 1;
    }

    // 解析命令行参数
    string osaFile = argv[1];
    string settingsPath = argv[2];
    string savePcdFile = "";
    string savePlyFile = "";
    string saveTrajectoryFile = "";
    float resolution = 0.01f;  // 默认下采样分辨率
    string vocabPath = "/home/wayuji/code/ORB_SLAM3/Vocabulary/ORBvoc.txt";  // 使用绝对路径
    
    for(int i = 3; i < argc; i++)
    {
        string arg = argv[i];
        if(arg == "--save-pcd" && i+1 < argc)
            savePcdFile = argv[++i];
        else if(arg == "--save-ply" && i+1 < argc)
            savePlyFile = argv[++i];
        else if(arg == "--save-trajectory" && i+1 < argc)
            saveTrajectoryFile = argv[++i];
        else if(arg == "--resolution" && i+1 < argc)
            resolution = stof(argv[++i]);
        else if(arg == "--vocabulary" && i+1 < argc)
            vocabPath = argv[++i];
    }
    
    cout << "正在读取地图: " << osaFile << endl;
    
    // 初始化ORB-SLAM3系统
    cout << "初始化ORB-SLAM3系统..." << endl;
    
    // 转换成绝对路径
    char absPathBuffer[PATH_MAX];
    if(realpath(osaFile.c_str(), absPathBuffer) == nullptr) {
        cerr << "\033[1;31m错误: 无法获取地图文件的绝对路径\033[0m" << endl;
        return 1;
    }
    string absPathOsa = string(absPathBuffer);
    cout << "使用地图文件的绝对路径: " << absPathOsa << endl;
    
    // 检测地图文件格式
    cout << "检测地图文件格式..." << endl;
    bool isTextFormat = false;
    {
        std::ifstream ifs(absPathOsa, std::ios::binary);
        if(!ifs.good()) {
            cerr << "\033[1;31m错误: 无法打开地图文件\033[0m" << endl;
            return 1;
        }

        // 先尝试二进制格式读取
        try {
            boost::archive::binary_iarchive ia(ifs);
            string strFileVoc;
            string strVocChecksum;
            ia >> strFileVoc;
            ia >> strVocChecksum;
            
            cout << "检测到二进制格式地图文件" << endl;
            isTextFormat = false;
        }
        catch(const std::exception& e) {
            // 如果二进制读取失败，重置文件指针并尝试文本格式
            ifs.clear();
            ifs.seekg(0, std::ios::beg);
            
            try {
                boost::archive::text_iarchive ia(ifs);
                string strFileVoc;
                string strVocChecksum;
                ia >> strFileVoc;
                ia >> strVocChecksum;
                
                cout << "检测到文本格式地图文件" << endl;
                isTextFormat = true;
            }
            catch(const std::exception& e) {
                cerr << "\033[1;31m错误: 无法确定地图文件格式\033[0m" << endl;
                return 1;
            }
        }
    }
    
    // 检查用户传入的设置文件是否已包含地图加载配置
    bool hasLoad = false;
    {
        ifstream fin(settingsPath);
        string line;
        while(getline(fin, line)) {
            if(line.find("System.LoadAtlasFromFile") != string::npos) {
                hasLoad = true; break;
            }
        }
    }

    // 始终使用命令行指定的地图文件覆盖配置文件中的设置
    ORB_SLAM3::System *pSLAM = nullptr;
    {
        string tempSettings = "/tmp/temp_read_settings.yaml";
        {
            ifstream fin(settingsPath);
            ofstream fout(tempSettings);
            fout << fin.rdbuf();
            fout << "\n# Override atlas load settings\n";
            fout << "System.LoadAtlasFromFile: " << absPathOsa << "\n";
            // 根据检测到的格式设置加载类型
            fout << "System.LoadFileType: " << (isTextFormat ? "TEXT" : "BINARY") << "\n";
        }
        cout << "使用临时设置文件: " << tempSettings << endl;
                pSLAM = new ORB_SLAM3::System(vocabPath, tempSettings, ORB_SLAM3::System::MONOCULAR, false, 0, "", true); // 启用只读模式 
    }

    // 获取点云可视化器
    ORB_SLAM3::PointCloudVisualizer* pVisualizer = pSLAM->GetPointCloudVisualizer();
    if(!pVisualizer)
    {
        cerr << "\033[1;31m错误: 无法获取点云可视化器\033[0m" << endl;
        return 1;
    }
    
    // 输出地图信息
    cout << "\033[1;32m正在获取地图信息...\033[0m" << endl;
    ORB_SLAM3::Atlas* pAtlas = pSLAM->GetAtlas();
    if(pAtlas)
    {
        vector<ORB_SLAM3::Map*> vpMaps = pAtlas->GetAllMaps();
        cout << "Atlas 包含 " << vpMaps.size() << " 个地图" << endl;
        
        for(size_t i=0; i<vpMaps.size(); i++)
        {
            ORB_SLAM3::Map* pMap = vpMaps[i];
            if(pMap)
            {
                cout << "地图 #" << i+1 << " (ID: " << pMap->GetId() << "):" << endl;
                cout << "  - 地图点数量: " << pMap->GetAllMapPoints().size() << endl;
                cout << "  - 关键帧数量: " << pMap->GetAllKeyFrames().size() << endl;
            }
            else
            {
                cout << "地图 #" << i+1 << " 无效" << endl;
            }
        }
    }
    else
    {
        cout << "\033[1;33m警告: Atlas 为空\033[0m" << endl;
    }
    
    // 创建PCL点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // 转换为PCL点云
    if (!ConvertToPCLPointCloud(pVisualizer, pointCloud))
    {
        pSLAM->Shutdown();
        delete pSLAM;
        return 1;
    }
    
    // 使用体素网格对点云进行下采样以减少点的数量
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
    voxelGrid.setInputCloud(pointCloud);
    voxelGrid.setLeafSize(resolution, resolution, resolution);
    voxelGrid.filter(*downsampledCloud);
    
    cout << "下采样后点云数量: " << downsampledCloud->points.size() << endl;
    
    // 保存PCD文件
    if(!savePcdFile.empty())
    {
        cout << "保存点云到PCD文件: " << savePcdFile << endl;
        pcl::io::savePCDFileASCII(savePcdFile, *downsampledCloud);
    }
    
    // 直接使用PointCloudVisualizer保存PLY文件
    if(!savePlyFile.empty())
    {
        cout << "保存点云到PLY文件: " << savePlyFile << endl;
        pVisualizer->SavePointCloudToPLY(savePlyFile);
    }
    
    // 保存轨迹文件
    if(!saveTrajectoryFile.empty())
    {
        cout << "保存轨迹到PLY文件: " << saveTrajectoryFile << endl;
        pVisualizer->SaveKeyFrameTrajectoryToPLY(saveTrajectoryFile);
    }
    
    // 可视化点云
    cout << "正在可视化点云..." << endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ORB_SLAM3 点云可视化"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB>(downsampledCloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    
    // 添加轨迹到可视化器
    AddTrajectoryToVisualizer(pVisualizer, viewer);
    
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    cout << "按 'q' 键退出可视化程序..." << endl;
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 关闭SLAM系统并释放资源
    pSLAM->Shutdown();
    delete pSLAM;

    return 0;
}