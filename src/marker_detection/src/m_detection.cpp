#include<iostream>
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/core/core.hpp>
#include<opencv4/opencv2/imgproc/imgproc.hpp>
#include"ros/ros.h"
#include"std_msgs/String.h"
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<sstream>
#include<string>
#include<fstream>
#include<boost/algorithm/string.hpp>

#include<jsoncpp/json/json.h>

int main(int argc, char** argv) {
    cv::VideoCapture inputVideo(0);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    if(!inputVideo.isOpened())
    {
        std::cout << "Error Opening video stream or file" << std::endl;
        return -1;
    }
    
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    std::string filename = "/home/cona/marker/src/marker_detection/src/cam_int";
    std::ifstream readFile;
    std::vector<std::string> yaml;
    std::vector<int> rows;
    std::vector<int> cols;
    std::vector<std::string> data;

    std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<cv::Vec3d> rvecs, tvecs;

    cv::Mat frame, frame_cp;
    cv::Mat camMatrix, distCoeffs;
    std::string parser;

    cv::FileStorage f_s(filename, cv::FileStorage::READ);

    if(!f_s.isOpened())
        std::cout<<"err"<<std::endl;
    
    f_s["camera_matrix"] >> camMatrix;
    f_s["distortion_coefficients"] >> distCoeffs;

    std::cout<<camMatrix<<std::endl;
    
    f_s.release();
    
    
    while (1)
    {
        inputVideo >> frame;
        frame.copyTo(frame_cp);
        if(frame.empty())
            break;
    
        cv::Mat image = cv::imread(argv[0], cv::IMREAD_COLOR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        ros::Rate loop_rate(10);
        if(nh.ok()) {
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }

        cv::aruco::detectMarkers(frame, dictionary, corners, ids);

        if(ids.size() > 0)
            cv::aruco::drawDetectedMarkers(frame_cp, corners, ids);

        cv::aruco::estimatePoseSingleMarkers(corners, 0.037, camMatrix, distCoeffs, rvecs, tvecs);
        // std::cout << rvecs.size() << std::endl;
        for (int i = 0; i < rvecs.size(); i ++)
        {
            std::cout << "rvecs[" << i << "] : "<< rvecs[i] << std::endl;
        }

        for (int i = 0; i < tvecs.size(); i ++)
        {
            std::cout << "tvecs[" << i << "] : "<< tvecs[i] << std::endl;
        }

        for (int i = 0; i < corners.size(); i++)
        {
            std::cout << "corners[" << i << "] : "<< corners[i] << std::endl;
        }

        for (int i=0; i<ids.size(); i++)
        {
            
            cv::drawFrameAxes(frame_cp, camMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            
        }

        cv::Mat flip_frame_cp;
        cv::flip(frame_cp, flip_frame_cp, 1);
        cv::imshow("flip", flip_frame_cp);
        cv::imshow("out", frame_cp);
        if(cv::waitKey(1)==27)
            break;
    }
    return 0;
}

// class ImageConverter
// {
//     ros::NodeHandle nh_;
//     image_transport::ImageTransport it_;
//     image_transport::Subscriber image_sub_;
//     image_transport::Publisher image_pub_;

//     public:
//         ImageConverter()
//             :it_(nh_)
//         {
//             image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
//             image_pub_ = it_.advertise("/image_converter/output_video", 1);
//         }
        
//         void imageCb(const sensor_msgs::ImageConstPtr& msg)
//         {
//             cv_bridge::CvImagePtr cv_ptr;
//             try {
//                 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             }
//             catch (cv_bridge::Exception& e) {
//                 ROS_ERROR("cv_bridge exception:%s", e.what());
//                 return;
//             }

//             if(cv_ptr->image.rows > 60 && cv_ptr -> image.cols > 60)
//                 cv::circle(cv_ptr -> image, cv::Point(50, 50), 10, CV_RGB(255,0, 0));

//                 cv::imshow("d", cv_ptr->image);
//                 cv::waitKey(0);

//                 image_pub_.publish(cv_ptr->toImageMsg());
//         }
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "image_converter");
//     ImageConverter ic;
//     ros::spin();
//     return 0;
// }