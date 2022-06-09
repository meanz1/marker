#include <iostream>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <string>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
// #include <tfMessage/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <deque>

cv::Mat frame;
cv::Mat frame_cp;
bool flag = false;
tf::Quaternion q;
tf::Quaternion q_;
// marker2cam (matrix)
tf::Matrix3x3 R_cam_link;
tf::Vector3 T_cam_link;

//adverse
tf::Matrix3x3 R_cam_link_ad;
tf::Vector3 T_cam_link_ad;
tf::Transform tf_base_msg;
// tf::StampedTransform base_st;
tf::Vector3 cv_vector3d_to_tf_vector3_m(const cv::Mat &vec)
{
    return {vec.at<double>(0, 0), vec.at<double>(1, 0), vec.at<double>(2, 0)};
}

tf::Quaternion cv_vector3d_to_tf_quaternion_m(const cv::Mat &rvec)
{
    cv::Mat rotation_mat;
    auto ax = rvec.at<double>(0, 0), ay = rvec.at<double>(1, 0), az = rvec.at<double>(2, 0);
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;

    q.setValue(qx, qy, qz, qw);
    return q;
}

tf::Transform create_transform_m(const cv::Mat &tvec, const cv::Mat &rvec)
{
    tf::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3_m(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion_m(rvec));
    return transform;
}

// cam2marker (cv::Vec ver)

tf::Vector3 cv_vector3d_to_tf_vector3_v(const cv::Vec3d &vec)
{
    return {vec[0], vec[1], vec[2]};
}

tf::Quaternion cv_vector3d_to_tf_quaternion_v(const cv::Vec3d &rvec)
{
    cv::Mat rotation_mat;
    auto ax = rvec[0], ay = rvec[1], az = rvec[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;

    q_.setValue(qx, qy, qz, qw);
    return q_;
}

tf::Transform create_transform_v(const cv::Vec3d &tvec, const cv::Vec3d &rvec)
{
    tf::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3_v(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion_v(rvec));
    return transform;
}

void imagecallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        ROS_INFO("Size : %d %d", msg->width, msg->height);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame = cv_ptr->image;
        frame.copyTo(frame_cp);
        if (frame.empty())
            std::cout << "frame empty" << std::endl;

        std::cout << frame.cols << " " << frame.rows << std::endl;

        flag = true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void cam_link2cam_optical()
{
    // tf_listener adverse of cam_link to cam_optical
    tf::StampedTransform tf_cam_msg;
    tf::StampedTransform tf_base2camera_link;
    tf::TransformListener listener_cam_link;
    
    bool flag_listener = listener_cam_link.waitForTransform("/camera_link", "/camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
    try
    {
        listener_cam_link.lookupTransform("/camera_link", "/camera_rgb_optical_frame", ros::Time(0), tf_cam_msg);

        R_cam_link = tf::Matrix3x3(tf_cam_msg.getRotation());
        T_cam_link = tf::Vector3(tf_cam_msg.getOrigin().x(), tf_cam_msg.getOrigin().y(), tf_cam_msg.getOrigin().z());
        std::cout << "T_cam_link : " << tf_cam_msg.getOrigin().x() << " " << tf_cam_msg.getOrigin().y() << " " << tf_cam_msg.getOrigin().z() << " " << std::endl;
    
        // adverse
        R_cam_link_ad = R_cam_link.inverse();

        cv::Mat rotation_cam_link_ad;

        for(int i = 0; i < 3; i ++)
        {
            for(int j = 0; j < 3; j++)
            {
                rotation_cam_link_ad.at<double>(i, j) = R_cam_link_ad.getRow(i)[j]; 
            }
        }
        rotation_cam_link_ad -= rotation_cam_link_ad;

        tf::Matrix3x3 R_cam_link_ad_(rotation_cam_link_ad.at<double>(0, 0), rotation_cam_link_ad.at<double>(0, 1), rotation_cam_link_ad.at<double>(0, 2), rotation_cam_link_ad.at<double>(1, 0), rotation_cam_link_ad.at<double>(1, 1), rotation_cam_link_ad.at<double>(1, 2), rotation_cam_link_ad.at<double>(2, 0), rotation_cam_link_ad.at<double>(2, 1), rotation_cam_link_ad.at<double>(2, 2));

        T_cam_link_ad = R_cam_link_ad_ * T_cam_link;
        tf::Transform t_base(R_cam_link_ad_, T_cam_link_ad);
        //auto t_base = create_transform_m(T_cam_link_ad, R_cam_link_ad);
        
        tf_base_msg.setOrigin(tf::Vector3(t_base.getOrigin().getX(), t_base.getOrigin().getY(), t_base.getOrigin().getZ()));
        tf_base_msg.setRotation(tf::Quaternion(t_base.getRotation().getX(), t_base.getRotation().getY(), t_base.getRotation().getZ(), t_base.getRotation().getW()));
        
        
        
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

int main(int argc, char **argv)
{
    // cv::VideoCapture inputVideo(0);

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.1485, 0.0135, dictionary);
    
    // for (int i = 0; i < 35; i++)
    // {
    //     std::cout << "board : " << board.objPoints[i][0] << board.objPoints[i][1] << board.objPoints[i][2] << board.objPoints[i][3] << std::endl;
    // }

    // if (!inputVideo.isOpened())
    // {
    //     std::cout << "Error Opening video stream or file" << std::endl;
    //     return -1;
    // }
    // inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    // inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    std::string filename = "/home/cona/marker/src/marker_detection/src/cam_int";

    // ros::Publisher tf_list_pub_ = nh.advertise<tf_msgs::TFMessage>("/tf", 10);
    std::deque<double> dqx;
    std::deque<double> dqy;
    std::deque<double> dqz;

    double dqx_ave = 0;
    double dqy_ave = 0;
    double dqz_ave = 0;

    std::string marker_tf_prefix;
    std::string marker_tf_prefix2;
    std::string marker_tf_prefix3;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Vec3d rvec_board, tvec_board;

    cv::Mat rvecs_inv;

    cv::Mat camMatrix, distCoeffs;

    tf::Transform transform;
    cv::Mat R;

    tf::TransformBroadcaster br;
    // tf_msgs::TFMessage tf_msg_list_;
    tf::TransformListener listener_base2camera_link;
    cv::FileStorage f_s(filename, cv::FileStorage::READ);

    if (!f_s.isOpened())
        std::cout << "err" << std::endl;

    f_s["camera_matrix"] >> camMatrix;
    f_s["distortion_coefficients"] >> distCoeffs;

    std::cout << camMatrix << std::endl;

    f_s.release();

    // image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imagecallback);
    // ros::spin();
    // while (ros::ok())
    // {
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imagecallback);
    tf::StampedTransform tf_b2c;
    tf::TransformListener listener;

    tf::TransformListener listener_cam;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (flag == true)
        {
            cv::aruco::detectMarkers(frame, dictionary, corners, ids);

            for (int i = 0; i < ids.size(); i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    std::cout << "[" << i << "] : " << corners[i][j] << std::endl;
                }
            }
            // cv::aruco::detectMarkers(frame, board.dictionary, corners, ids);

            if (ids.size() > 0)
            {
                cv::aruco::drawDetectedMarkers(frame_cp, corners, ids);
                // std::cout << corners << std::endl;

                int valid = cv::aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec_board, tvec_board);

                if (valid > 0)
                {
                    cv::drawFrameAxes(frame_cp, camMatrix, distCoeffs, rvec_board, tvec_board, 0.1);
                }
            }
            // cv::aruco::estimatePoseSingleMarkers(corners, 0.2, camMatrix, distCoeffs, rvecs, tvecs);

            cv::Mat R_inv, T_inv;
            cv::Rodrigues(rvec_board, R);
            R_inv = R.inv();
            cv::Rodrigues(R_inv, rvecs_inv);
            T_inv = -R_inv * tvec_board;

            std::cout << "c2m distance" << std::sqrt(tvec_board[0] * tvec_board[0] + tvec_board[1] * tvec_board[1] + tvec_board[2] * tvec_board[2]) << std::endl;

            // cam2marker

            auto transform_v = create_transform_v(tvec_board, rvec_board); // c2m
            geometry_msgs::TransformStamped tf_msg_v;
            tf_msg_v.header.stamp = ros::Time::now();
            tf_msg_v.header.frame_id = "camera_rgb_optical_frame";
            std::stringstream ss;
            ss << marker_tf_prefix << "marker__";
            tf_msg_v.child_frame_id = ss.str();
            tf_msg_v.transform.translation.x = transform_v.getOrigin().getX();
            tf_msg_v.transform.translation.y = transform_v.getOrigin().getY();
            tf_msg_v.transform.translation.z = transform_v.getOrigin().getZ();
            std::cout << "x : " << transform_v.getOrigin().getX() << " y : " << transform_v.getOrigin().getY() << " z : " << transform_v.getOrigin().getZ() << std::endl;
            tf_msg_v.transform.rotation.x = transform_v.getRotation().getX();
            tf_msg_v.transform.rotation.y = transform_v.getRotation().getY();
            tf_msg_v.transform.rotation.z = transform_v.getRotation().getZ();
            tf_msg_v.transform.rotation.w = transform_v.getRotation().getW();

            // marker2cam

            auto transform_m = create_transform_m(T_inv, rvecs_inv); // m2c
            // geometry_msgs::TransformStamped tf_msg_m;
            tf::Transform tf_msg_m;
            
            tf_msg_m.setOrigin(tf::Vector3(transform_m.getOrigin().getX(), transform_m.getOrigin().getY(), transform_m.getOrigin().getZ()));
            tf_msg_m.setRotation(tf::Quaternion(transform_m.getRotation().getX(), transform_m.getRotation().getY(), transform_m.getRotation().getZ(), transform_m.getRotation().getW()));
            
            br.sendTransform(tf_msg_v);
            tf::StampedTransform tf_st_msg_m(tf_msg_m, ros::Time::now(), "marker", "cam");
            br.sendTransform(tf_st_msg_m);

            tf::StampedTransform tf_m;
            tf::StampedTransform cam_optical2cam_link;

            cam_link2cam_optical();
            tf::StampedTransform base_st(tf_base_msg, ros::Time::now(), "camera_link", "b_test");

            br.sendTransform(base_st);

            bool output = false;
            bool scs = listener.waitForTransform("/base_link", "/cam", ros::Time(0), ros::Duration(1.0));
            if (!scs)
                return output;
            listener.lookupTransform("/base_link", "/cam", ros::Time(0), tf_m);

            // bool cam_optical_s = listener_cam.waitForTransform("/")
            bool aaa = listener_base2camera_link.waitForTransform("/base_link", "/b_test", ros::Time(0), ros::Duration(1.0));
            if (!aaa)
                return output;
            listener_base2camera_link.lookupTransform("/base_link", "b_test", ros::Time(0), cam_optical2cam_link);
            
            if (dqz.size() >= 10)
            {
                dqz.pop_front();
            }

            if (dqx.size() >= 10)
            {
                dqx.pop_front();
            }

            if (dqy.size() >= 10)
            {
                dqy.pop_front();
            }

            dqx.push_back(tf_m.getOrigin().getX());
            dqy.push_back(tf_m.getOrigin().getY());
            dqz.push_back(tf_m.getOrigin().getZ());

            if (dqx.size() == 10)
            {
                std::ofstream value_text;
                value_text.open("/home/cona/marker/src/marker_detection/src/value.txt");

                double dqx_sum = 0;
                double dqy_sum = 0;
                double dqz_sum = 0;

                for (int i = 0; i < dqz.size(); i++)
                {
                    dqz_sum += dqz.at(i);
                }
                dqz_ave = dqz_sum / dqz.size();

                for (int i = 0; i < dqx.size(); i++)
                {
                    dqx_sum += dqx.at(i);
                }
                dqx_ave = dqx_sum / dqx.size();

                for (int i = 0; i < dqy.size(); i++)
                {
                    dqy_sum += dqy.at(i);
                }
                dqy_ave = dqy_sum / dqy.size();

                std::cout << "tf_m X : " << tf_m.getOrigin().getX() << " Y : " << tf_m.getOrigin().getY() << " Z : " << tf_m.getOrigin().getZ() << std::endl;
                std::cout << "tf_m X : " << tf_m.getRotation().getX() << " Y : " << tf_m.getRotation().getY() << " Z : " << tf_m.getRotation().getZ() << " W : " << tf_m.getRotation().getW() << std::endl;

                geometry_msgs::TransformStamped tf_m_msg;
                tf_m_msg.header.stamp = ros::Time::now();
                tf_m_msg.header.frame_id = "base_link";
                std::stringstream sss;
                sss << marker_tf_prefix3 << "test";
                tf_m_msg.child_frame_id = sss.str();
                tf_m_msg.transform.translation.x = tf_m.getOrigin().getX();
                tf_m_msg.transform.translation.y = tf_m.getOrigin().getY();
                tf_m_msg.transform.translation.z = tf_m.getOrigin().getZ();

                tf_m_msg.transform.rotation.x = tf_m.getRotation().getX();
                tf_m_msg.transform.rotation.y = tf_m.getRotation().getY();
                tf_m_msg.transform.rotation.z = tf_m.getRotation().getZ();
                tf_m_msg.transform.rotation.w = tf_m.getRotation().getW();
                br.sendTransform(tf_m_msg);
                // br.sendTransform(tf_b2c);

                tf::Quaternion value_rpy;
                value_rpy.setValue(tf_m.getRotation().getX(), tf_m.getRotation().getY(), tf_m.getRotation().getZ(), tf_m.getRotation().getW());
                tf::Matrix3x3 m(value_rpy);
                double roll = 0.0, pitch = 0.0, yaw = 0.0;
                m.getRPY(roll, pitch, yaw);
                std::string str = "TRANSFORMATION x : " + std::to_string(dqx_ave) + " y : " + std::to_string(dqy_ave) + " z : " + std::to_string(dqz_ave);
                std::string str_r = " ROTATION r : " + std::to_string(roll) + " p : " + std::to_string(pitch) + " y : " + std::to_string(yaw);

                value_text.write(str.c_str(), str.size());
                value_text.write(str_r.c_str(), str_r.size());
                value_text.close();
            }

            // broadcast tf_msg
            // br.sendTransform(tf_msg_v);
            // br.sendTransform(tf_msg_m);
            // br.sendTransform(tf::StampedTransform(tf_msg_m, ros::Time::now(), "marker", "cam"));
            cv::circle(frame_cp, cv::Point(344, 216), 10, cv::Scalar(255, 0, 0));
            flag = false;
            cv::imshow("out", frame_cp);
            if (cv::waitKey(1) == 27)
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
