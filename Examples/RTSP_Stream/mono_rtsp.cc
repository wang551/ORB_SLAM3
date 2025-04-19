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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include<queue>
#include<mutex>
#include<condition_variable>
#include<signal.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<System.h>
// 添加必要的头文件
#include<Map.h>
#include<MapPoint.h>
#include<Atlas.h>
#include<KeyFrame.h>

using namespace std;

// 帧缓冲队列的最大容量
const int MAX_BUFFER_SIZE = 5;

// 帧缓冲结构
struct FrameBuffer {
    cv::Mat frame;
    double timestamp;
};

// 全局变量
queue<FrameBuffer> frameQueue;
mutex queueMutex;
condition_variable frameCondition;
bool isRunning = true;  // 控制程序运行
int skipFrames = 0;  // 每处理一帧后跳过的帧数

void ProcessFrame(ORB_SLAM3::System &SLAM, cv::Mat frame, double tframe, double &ttrack, float imageScale);
void FrameGrabber(const string& url);

// 保存点云数据的函数
bool SavePointCloudData(ORB_SLAM3::System &SLAM, const string &filename) {
    // 使用互斥锁保护保存操作
    lock_guard<mutex> lock(saveMutex);
    
    try {
        // 首先检查SLAM系统是否已经关闭
        if(SLAM.isShutDown()) {
            cerr << "SLAM系统已关闭，无法保存点云数据" << endl;
            return false;
        }

        cout << "正在保存点云数据到 " << filename << " ..." << endl;
        
        // 获取当前跟踪的地图点
        vector<ORB_SLAM3::MapPoint*> vpMapPoints;
        try {
            vpMapPoints = SLAM.GetTrackedMapPoints();
        } catch(const exception& e) {
            cerr << "获取地图点时出错: " << e.what() << endl;
            return false;
        }
        
        if(vpMapPoints.empty()) {
            cerr << "当前没有跟踪的地图点，无法保存点云数据！" << endl;
            return false;
        }
        
        // 创建文件并保存点云数据
        ofstream f;
        f.open(filename.c_str());
        if(!f.is_open()) {
            cerr << "无法创建文件 " << filename << endl;
            return false;
        }

        f << fixed;
        
        // 统计有效的点数量
        int validPointCount = 0;
        for(auto* pMP : vpMapPoints) {
            if(pMP && !pMP->isBad()) {
                validPointCount++;
            }
        }
        
        if(validPointCount == 0) {
            cerr << "没有有效的地图点，无法保存点云数据！" << endl;
            f.close();
            return false;
        }
        
        // 保存为 PLY 格式
        f << "ply" << endl;
        f << "format ascii 1.0" << endl;
        f << "element vertex " << validPointCount << endl;
        f << "property float x" << endl;
        f << "property float y" << endl;
        f << "property float z" << endl;
        f << "property uchar red" << endl;
        f << "property uchar green" << endl;
        f << "property uchar blue" << endl;
        f << "end_header" << endl;

        // 写入点云数据
        int validPoints = 0;
        for(size_t i=0; i<vpMapPoints.size(); i++) {
            ORB_SLAM3::MapPoint* pMP = vpMapPoints[i];
            if(pMP && !pMP->isBad()) {
                try {
                    Eigen::Vector3f pos = pMP->GetWorldPos();
                    
                    // 使用默认颜色（可以根据需要修改）
                    int r = 0, g = 255, b = 0; // 默认绿色
                    
                    f << setprecision(6) << pos(0) << " " << pos(1) << " " << pos(2)
                      << " " << r << " " << g << " " << b << endl;
                      
                    validPoints++;
                } catch(const exception& e) {
                    // 忽略单个点的错误，继续处理其他点
                    continue;
                }
            }
        }
        
        f.close();
        cout << "成功保存了 " << validPoints << " 个点云数据点到 " << filename << endl;
        return true;
    }
    catch(const exception& e) {
        cerr << "保存点云数据时出错: " << e.what() << endl;
        return false;
    }
}

// 处理外部停止信号的函数
void SignalHandler(int signum) {
    cout << "Shutdown signal received, stopping safely..." << endl;
    isRunning = false;
    frameCondition.notify_all();  // 确保所有等待的条件变量都被通知
}

int main(int argc, char **argv)
{
    if(argc < 4 || argc > 5)
    {
        cerr << endl << "Usage: ./mono_rtsp path_to_vocabulary path_to_settings RTSP_stream_Url [skipFrames]" << endl;
        return 1;
    }

    // 设置信号处理函数来捕获CTRL+C和系统终止信号
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
    
    const string Url = string(argv[3]);
    
    // 可选的跳帧参数
    if(argc == 5)
        skipFrames = atoi(argv[4]);

    cout << "RTSP URL: " << Url << endl;
    if(skipFrames > 0)
        cout << "Frame skipping enabled: Processing 1 frame and skipping " << skipFrames << " frames" << endl;

    // 创建SLAM系统。它初始化所有系统线程并准备处理帧
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    // 跟踪时间统计向量
    vector<float> vTimesTrack;
    vector<double> vTimestamps;

    cout << endl << "-------" << endl;
    cout << "Start processing RTSP stream ..." << endl;
    
    // 启动帧抓取线程
    thread grabberThread(FrameGrabber, Url);
    
    int ni = 0;
    FrameBuffer currentFrame;
    
    // 主处理循环
    while(isRunning)
    {
        // 从队列获取帧并检查系统状态
        {
            unique_lock<mutex> lock(queueMutex);
            frameCondition.wait(lock, [&SLAM]{return !frameQueue.empty() || !isRunning || SLAM.isShutDown();});
            
            // 检查退出条件
            if((!isRunning && frameQueue.empty()) || SLAM.isShutDown())
            {
                if(SLAM.isShutDown()) {
                    cout << "SLAM系统已关闭，退出处理循环" << endl;
                    isRunning = false;
                }
                break;
            }
            
            // 如果队列为空但程序仍在运行，继续等待
            if(frameQueue.empty())
                continue;
            
            // 获取一帧用于处理
            currentFrame = frameQueue.front();
            frameQueue.pop();
        }
        
        double ttrack = 0.f;
        
        try {
            // 处理帧
            ProcessFrame(SLAM, currentFrame.frame, currentFrame.timestamp, ttrack, imageScale);
            
            vTimestamps.push_back(currentFrame.timestamp);
            vTimesTrack.push_back(ttrack);
        }
        catch(const exception& e) {
            cerr << "Exception during frame processing: " << e.what() << endl;
            break;
        }
        
        // 显示实时跟踪性能
        if(ni % 30 == 0) { // 每30帧输出一次
            double avgTrackTime = 0;
            int recentFrames = min(30, ni);
            for(int i = 0; i < recentFrames; i++)
                avgTrackTime += vTimesTrack[vTimesTrack.size()-1-i];
            avgTrackTime /= recentFrames;
            
            cout << "Frame: " << ni << ", Queue size: " << frameQueue.size() 
                 << ", Avg tracking time: " << avgTrackTime * 1000 << "ms" << endl;
        }
        
        ni++;
    }
    
    cout << "程序退出中..." << endl;
    
    // 通知并等待帧抓取线程结束
    isRunning = false;
    frameCondition.notify_all(); 
    if(grabberThread.joinable()) {
        cout << "等待帧抓取线程结束..." << endl;
        grabberThread.join();
        cout << "帧抓取线程已结束." << endl;
    }
    
    try {
        // 确保队列已清空
        {
            unique_lock<mutex> lock(queueMutex);
            while(!frameQueue.empty())
                frameQueue.pop();
        }

        // 在SLAM系统还没被关闭之前，尝试保存点云数据
        if(!SLAM.isShutDown()) {
            // 先保存点云，再让系统退出时保存Atlas
            string pointCloudFile = "PointCloud.ply";
            cout << "正在尝试保存点云数据..." << endl;
            if(SavePointCloudData(SLAM, pointCloudFile)) {
                cout << "点云数据已保存至 " << pointCloudFile << endl;
            } else {
                cout << "保存点云数据失败，将仅通过Atlas保存地图" << endl;
            }
        } else {
            cout << "SLAM系统已关闭，无法保存点云数据" << endl;
        }
        
        // 跟踪时间统计
        if(!vTimesTrack.empty())
        {
            sort(vTimesTrack.begin(), vTimesTrack.end());
            float totaltime = 0;
            int nFrames = vTimesTrack.size();
            for(int i=0; i<nFrames; i++)
                totaltime += vTimesTrack[i];
                
            cout << "-------" << endl << endl;
            cout << "Frames processed: " << nFrames << endl;
            cout << "median tracking time: " << vTimesTrack[nFrames/2] * 1000 << "ms" << endl;
            cout << "mean tracking time: " << totaltime/nFrames * 1000 << "ms" << endl;
        }
    }
    catch(const exception& e) {
        cerr << "Exception during shutdown: " << e.what() << endl;
    }
    
    cout << "程序正常退出." << endl;
    return 0;
}

// 帧抓取线程函数
void FrameGrabber(const string& url)
{
    cv::VideoCapture cap(url, cv::CAP_FFMPEG);
    
    // 设置RTSP缓冲大小为最小值，减少延迟
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    
    if (!cap.isOpened()) {
        cerr << "Error: Could not open RTSP stream!" << endl;
        isRunning = false;
        frameCondition.notify_one();  // 通知主线程
        return;
    }
    cout << "RTSP stream opened successfully!" << endl;
    
    int frameCount = 0;
    int skippedFrames = 0;
    
    while(isRunning)
    {
        cv::Mat frame;
        if (!cap.read(frame)) {
            cerr << "Connection lost, reconnecting..." << endl;
            cap.release();
            // 重新连接前短暂延迟
            this_thread::sleep_for(chrono::milliseconds(500));
            
            // 检查是否应该停止
            if(!isRunning)
                break;
                
            cap.open(url, cv::CAP_FFMPEG);  // 使用FFMPEG而不是GStreamer确保更好的兼容性
            cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
            continue;
        }
        
        frameCount++;
        
        // 实现跳帧逻辑
        if(skipFrames > 0 && skippedFrames < skipFrames) {
            skippedFrames++;
            continue;
        }
        skippedFrames = 0;
        
        // 获取当前时间戳
        auto timestamp = chrono::system_clock::now();
        auto duration = timestamp.time_since_epoch();
        double tframe = chrono::duration_cast<chrono::milliseconds>(duration).count() / 1000.0;
        
        // 将帧加入队列
        try {
            unique_lock<mutex> lock(queueMutex);
            
            // 检查是否应该停止
            if(!isRunning)
                break;
                
            // 如果队列满，移除最老的帧
            while(frameQueue.size() >= MAX_BUFFER_SIZE) {
                frameQueue.pop();
            }
            
            // 添加新帧到队列
            FrameBuffer buffer = {frame.clone(), tframe};
            frameQueue.push(buffer);
            
            // 通知处理线程
            frameCondition.notify_one();
        }
        catch(const exception& e) {
            cerr << "Exception in frame grabber: " << e.what() << endl;
            break;
        }
    }
    
    cap.release();
    cout << "Frame grabber stopped" << endl;
}

void ProcessFrame(ORB_SLAM3::System &SLAM, cv::Mat frame, double tframe, double &ttrack, float imageScale)
{
    double t_resize = 0.f;
    double t_track = 0.f;

    if (frame.empty())
    {
        cerr << endl << "Frame is empty!" << endl;
        return;
    }

    // 性能优化：在进行SLAM处理前，可以考虑降低分辨率
    cv::Mat processFrame;
    if(imageScale != 1.f)
    {
    #ifdef REGISTER_TIMES
        #if __cplusplus >= 201103L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201103L) || defined(COMPILEDWITHC11) || defined(COMPILEDWITHC14)
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
        #endif
    #endif
        int width = frame.cols * imageScale;
        int height = frame.rows * imageScale;
        cv::resize(frame, processFrame, cv::Size(width, height));
    #ifdef REGISTER_TIMES
        #if __cplusplus >= 201103L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201103L) || defined(COMPILEDWITHC11) || defined(COMPILEDWITHC14)
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
        #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
        #endif
        t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
        SLAM.InsertResizeTime(t_resize);
    #endif
    }
    else {
        processFrame = frame;
    }

    #if __cplusplus >= 201103L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201103L) || defined(COMPILEDWITHC11) || defined(COMPILEDWITHC14)
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

    // 将图像传递给SLAM系统，这里不检查SLAM.isShutDown()，
    // 让系统内部处理关闭状态
    SLAM.TrackMonocular(processFrame, tframe);

    #if __cplusplus >= 201103L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201103L) || defined(COMPILEDWITHC11) || defined(COMPILEDWITHC14)
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

    #ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
    #endif

    ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
}