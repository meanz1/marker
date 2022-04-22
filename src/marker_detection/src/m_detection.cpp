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
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>
#include<jsoncpp/json/json.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2/LinearMath/Vector3.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2_msgs/TFMessage.h>

tf2::Vector3 cv_vector3d_to_tf_vector3(const cv::Vec3d &vec)
{
    return {vec[0], vec[1], vec[2]};
}

tf2::Quaternion cv_vector3d_to_tf_quaternion(const cv::Vec3d &rvec)
{
    cv::Mat rotation_mat;
    auto ax = rvec[0], ay = rvec[1], az = rvec[2];
    auto angle = sqrt(ax*ax + ay*ay + az*az);
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;
    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);
    return q;
}

tf2::Transform create_transform(const cv::Vec3d &tvec, const cv::Vec3d &rvec)
{
    tf2::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion(rvec));
    return transform;
}


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
    //ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseArray>("/markerPose", 10);
    ros::Publisher pos__ = nh.advertise<geometry_msgs::Pose>("/posem", 10);
    ros::Publisher tf_list_pub_ = nh.advertise<tf2_msgs::TFMessage>("/tf_list", 10);
    std::ifstream readFile;
    std::vector<std::string> yaml;
    std::vector<int> rows;
    std::vector<int> cols;
    std::vector<std::string> data;
    std::string marker_tf_prefix;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::Mat frame, frame_cp;
    cv::Mat camMatrix, distCoeffs;
    std::string parser;

    geometry_msgs::PoseArray posearray;

    cv::FileStorage f_s(filename, cv::FileStorage::READ);

    if(!f_s.isOpened())
        std::cout<<"err"<<std::endl;
    
    f_s["camera_matrix"] >> camMatrix;
    f_s["distortion_coefficients"] >> distCoeffs;

    std::cout<<camMatrix<<std::endl;
    
    f_s.release();
    
    
    while (1)
    {

        posearray.header.stamp = ros::Time::now();
        posearray.header.frame_id = "marker";
        
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
            std::cout << "rvecs[" << i << "] : "<< rvecs[i][0] << std::endl;
        }
        geometry_msgs::Pose p;

        tf2_ros::TransformBroadcaster br;
        tf2_msgs::TFMessage tf_msg_list_;

        for (int i = 0; i < tvecs.size(); i ++)
        {
            std::cout << "tvecs[" << i << "] : "<< tvecs[i] << std::endl;

            
            tf2::Quaternion marker_q;
            marker_q.setRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]);
            marker_q = marker_q.normalize();

            p.position.x = tvecs[i][0];
            p.position.y = tvecs[i][1];
            p.position.z = tvecs[i][2];

            p.orientation.x = marker_q.getX();
            p.orientation.y = marker_q.getY();
            p.orientation.z = marker_q.getZ();
            p.orientation.w = marker_q.getW();

            auto translation_vec = tvecs[i];
            auto rotation_vec = rvecs[i];
            auto transform = create_transform(translation_vec, rotation_vec);

            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = ros::Time::now();
            tf_msg.header.frame_id = "marker";

            std::stringstream ss;
            ss << marker_tf_prefix << ids[i];
            tf_msg.child_frame_id = ss.str();

            tf_msg.transform.translation.x = transform.getOrigin().getX();
            tf_msg.transform.translation.y = transform.getOrigin().getY();
            tf_msg.transform.translation.z = transform.getOrigin().getZ();

            tf_msg.transform.rotation.x = transform.getRotation().getX();
            tf_msg.transform.rotation.y = transform.getRotation().getY();
            tf_msg.transform.rotation.z = transform.getRotation().getZ();
            tf_msg.transform.rotation.w = transform.getRotation().getW();
            tf_msg_list_.transforms.push_back(tf_msg);
            br.sendTransform(tf_msg);

            //posearray.poses.push_back(p);
        }
        tf_list_pub_.publish(tf_msg_list_);


        pos__.publish(p);
     






        for (int i = 0; i < corners.size(); i++)
        {
            std::cout << "corners[" << i << "] : "<< corners[i] << std::endl;
        }

        for (int i=0; i<ids.size(); i++)
        {
            cv::drawFrameAxes(frame_cp, camMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }
        
        cv::imshow("out", frame_cp);
        if(cv::waitKey(1)==27)
            break;
    }
    return 0;
}
