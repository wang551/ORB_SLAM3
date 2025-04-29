#include<iostream>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<System.h>

using namespace std;

void ProcessFrame(ORB_SLAM3::System &SLAM, cv::Mat frame, double tframe, double &ttrack, float imageScale);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_video path_to_vocabulary path_to_settings video_file" << endl;
        return 1;
    }

    const string videoFile = string(argv[3]);
    cv::VideoCapture cap(videoFile);
    
    if (!cap.isOpened()) {
        cerr << "Error: Could not open video file!" << endl;
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    cout << endl << "-------" << endl;
    cout << "Start processing video ..." << endl;

    int ni = 0;
    double fps = cap.get(cv::CAP_PROP_FPS);
    double frameInterval = 1.0/fps;
    double lastTimestamp = 0;

    while(true)
    {
        cv::Mat frame;
        if (!cap.read(frame)) {
            cout << "Video processing finished!" << endl;
            break;
        }

        // 生成精确时间戳（替代相机时间戳）
        double tframe = lastTimestamp + frameInterval;
        lastTimestamp = tframe;

        double ttrack = 0.f;
        ProcessFrame(SLAM, frame, tframe, ttrack, imageScale);

        // 按ESC键退出
        char c = (char)cv::waitKey(1);
        if(c == 27) break;

        ni++;
        if(ni % 30 == 0) {
            cout << "Frame: " << ni << ", tracking time: " << ttrack * 1000 << "ms" << endl;
        }
    }

    // 保存结果
    string pointCloudFile = "PointCloud.ply";

    cap.release();
    return 0;
}

// 保留原有的ProcessFrame函数
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