#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main() {
    // 读取深度图像的路径
    string imagePath = "/home/gtf/dataset/rgbd_dataset_freiburg1_desk/depth/1305031453.374112.png";

    // 读取深度图像，使用 IMREAD_UNCHANGED 确保类型不被修改
    Mat depthImage = imread(imagePath, IMREAD_UNCHANGED);

    // 检查图像是否成功加载
    if (depthImage.empty()) {
        cerr << "Failed to load image at: " << imagePath << endl;
        return -1;
    }

    // 输出图像类型
    cout << "Image type: " << depthImage.type() << endl;

    double minVal, maxVal;
    cv::minMaxLoc(depthImage, &minVal, &maxVal);
    cout << "Depth range: [" << minVal << ", " << maxVal << "] meters  "<< endl;

    // 检查图像类型并读取(300, 300)的深度值
    float depthValue = 0.0f;
    if (depthImage.type() == CV_16UC1) {  // 16位无符号整数
        uint16_t pixelValue = depthImage.at<uint16_t>(300, 300);
        depthValue = pixelValue * 0.001f; // 转换为米（假设单位是毫米）
        cout << "Depth at (300, 300): " << pixelValue << " mm, " << depthValue << " meters" << endl;
    }
    else if (depthImage.type() == CV_32FC1) {  // 32位浮点数
        depthValue = depthImage.at<float>(300, 300);
        cout << "Depth at (300, 300): " << depthValue << " meters" << endl;
    }
    else {
        cerr << "Unsupported image type: " << depthImage.type() << endl;
        return -1;
    }

    return 0;
}
