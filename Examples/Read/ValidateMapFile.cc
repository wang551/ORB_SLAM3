#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<boost/archive/binary_iarchive.hpp>
#include<boost/archive/text_iarchive.hpp>
#include<boost/serialization/string.hpp>

using namespace std;

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        cout << "用法: ./ValidateMapFile path_to_file.osa" << endl;
        return 1;
    }

    string osaFile = argv[1];
    cout << "检验地图文件: " << osaFile << endl;

    // 尝试打开文件并读取头部信息
    try {
        std::ifstream ifs(osaFile, std::ios::binary);
        if(!ifs.good()) {
            cerr << "错误: 无法打开文件!" << endl;
            return 1;
        }

        // 先尝试二进制格式读取
        cout << "尝试以二进制格式读取文件..." << endl;
        try {
            boost::archive::binary_iarchive ia(ifs);
            string strFileVoc;
            string strVocChecksum;
            ia >> strFileVoc;
            ia >> strVocChecksum;
            
            cout << "成功读取头部信息!" << endl;
            cout << "词汇文件名: " << strFileVoc << endl;
            cout << "词汇校验和: " << strVocChecksum << endl;
            return 0;
        }
        catch(const std::exception& e) {
            cout << "二进制格式读取失败: " << e.what() << endl;
        }

        // 如果二进制格式失败，重置文件指针并尝试文本格式
        ifs.clear();
        ifs.seekg(0, std::ios::beg);
        
        cout << "尝试以文本格式读取文件..." << endl;
        try {
            boost::archive::text_iarchive ia(ifs);
            string strFileVoc;
            string strVocChecksum;
            ia >> strFileVoc;
            ia >> strVocChecksum;
            
            cout << "成功读取头部信息!" << endl;
            cout << "词汇文件名: " << strFileVoc << endl;
            cout << "词汇校验和: " << strVocChecksum << endl;
            return 0;
        }
        catch(const std::exception& e) {
            cout << "文本格式读取失败: " << e.what() << endl;
        }

        cout << "无法识别的文件格式!" << endl;
        return 1;
    }
    catch(const std::exception& e) {
        cerr << "出错: " << e.what() << endl;
        return 1;
    }

    return 0;
}