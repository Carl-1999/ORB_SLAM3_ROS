#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <sstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "System.h"
#include <opencv2/core/core.hpp>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ros::NodeHandle& nh)
        : mpSLAM(pSLAM)
    {
        // 创建图像发布器，发布到新的话题
        image_pub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_processed", 1);
    }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
    ros::Publisher image_pub;
};

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

class ImagePub
{
    public:
        ImagePub(const string& strSequence, image_transport::Publisher& imgPub)
            : strSequence(strSequence), image_pub_(imgPub) {}

        void LoadImages();
        void PubliserImages();
        
        string strSequence;
        image_transport::Publisher image_pub_;
        vector<string> vstrImageFilenames;
        vector<double> vTimestamps;
};

long spinCnt = 0;
double t_temp = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }    

    string voc_dir = argv[2];
    string config_dir = argv[1];
    string strSeq = argv[3];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_dir, config_dir, ORB_SLAM3::System::MONOCULAR, false);
    ros::NodeHandle nodeHandler;

    // 创建 ImageGrabber 实例并订阅图像话题
    ImageGrabber igb(&SLAM, nodeHandler);
    ros::Subscriber sub = nodeHandler.subscribe("/camera/rgb/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // 处理接收到的图像
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    
    // 将 ROS 图像消息转换为 OpenCV 图像
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double timread = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    // 使用 SLAM 进行单目跟踪
    mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

    // 修改时间戳
    ros::Time new_time = ros::Time::now();  // 或者您可以使用任何自定义的时间戳
    sensor_msgs::Image processed_image = *cv_ptr->toImageMsg();  // 获取消息副本

    // 设置自定义时间戳
    processed_image.header.stamp = new_time;

    // 发布图像消息
    image_pub.publish(processed_image);
    std::cout<<"GrabImage"<<endl;
}
