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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <deque>

cv::Mat frame;
cv::Mat frame_cp;
bool flag = false;
tf2::Quaternion q;
tf2::Quaternion q_;
// marker2cam (matrix)

tf2::Vector3 cv_vector3d_to_tf_vector3_m(const cv::Mat &vec)
{
    return {vec.at<double>(0, 0), vec.at<double>(1, 0), vec.at<double>(2, 0)};
}

tf2::Quaternion cv_vector3d_to_tf_quaternion_m(const cv::Mat &rvec)
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

tf2::Transform create_transform_m(const cv::Mat &tvec, const cv::Mat &rvec)
{
    tf2::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3_m(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion_m(rvec));
    return transform;
}

// cam2marker (cv::Vec ver)

tf2::Vector3 cv_vector3d_to_tf_vector3_v(const cv::Vec3d &vec)
{
    return {vec[0], vec[1], vec[2]};
}

tf2::Quaternion cv_vector3d_to_tf_quaternion_v(const cv::Vec3d &rvec)
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

tf2::Transform create_transform_v(const cv::Vec3d &tvec, const cv::Vec3d &rvec)
{
    tf2::Transform transform;
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

    ros::Publisher tf_list_pub_ = nh.advertise<tf2_msgs::TFMessage>("/tf", 10);
    std::deque<double> dqx;
    std::deque<double> dqy;
    std::deque<double> dqz;

    double dqx_ave = 0;
    double dqy_ave = 0;
    double dqz_ave = 0;

    std::string marker_tf_prefix;
    std::string marker_tf_prefix2;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Vec3d rvec_board, tvec_board;

    cv::Mat rvecs_inv;

    cv::Mat camMatrix, distCoeffs;

    tf2::Transform transform;
    cv::Mat R;

    tf2_ros::TransformBroadcaster br;
    tf2_msgs::TFMessage tf_msg_list_;

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
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (flag == true)
        {
            cv::aruco::detectMarkers(frame, dictionary, corners, ids);
            
            for (int i = 0; i < ids.size(); i++)
            {
                for (int j = 0; j < 4; j ++)
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
            geometry_msgs::TransformStamped tf_msg_m;
            tf_msg_m.header.stamp = ros::Time::now();
            tf_msg_m.header.frame_id = "marker";
            std::stringstream mss;
            mss << marker_tf_prefix2 << "cam";
            tf_msg_m.child_frame_id = mss.str();

            tf_msg_m.transform.translation.x = transform_m.getOrigin().getX();
            tf_msg_m.transform.translation.y = transform_m.getOrigin().getY();
            tf_msg_m.transform.translation.z = transform_m.getOrigin().getZ();

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

            dqx.push_back(tf_msg_m.transform.translation.x);
            dqy.push_back(tf_msg_m.transform.translation.y);
            dqz.push_back(tf_msg_m.transform.translation.z);

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

    
    
            std::cout << "x : " << transform_m.getOrigin().getX() << " y : " << transform_m.getOrigin().getY() << " z : " << transform_m.getOrigin().getZ() << std::endl;
            std::cout << "dqx ave : " << dqx_ave << std::endl;
            std::cout << "dqx size : " << dqx.size() << std::endl;
            std::cout << "dqy ave : " << dqy_ave << std::endl;
            std::cout << "dqy size : " << dqy.size() << std::endl;
            std::cout << "dqz ave : " << dqz_ave << std::endl;
            std::cout << "dqz size : " << dqz.size() << std::endl;

            tf_msg_m.transform.rotation.x = transform_m.getRotation().getX();
            tf_msg_m.transform.rotation.y = transform_m.getRotation().getY();
            tf_msg_m.transform.rotation.z = transform_m.getRotation().getZ();
            tf_msg_m.transform.rotation.w = transform_m.getRotation().getW();

            if (dqx.size() == 10)
            {
                std::ofstream value_text;
                value_text.open("/home/cona/marker/src/marker_detection/src/value.txt");
                std::string str = "TRANSFORMATION x : " + std::to_string(dqx_ave) + " y : " + std::to_string(dqy_ave) + " z : " + std::to_string(dqz_ave);

                // double sinr_cosp = 2 * (tf_msg_v.transform.rotation.w * tf_msg_v.transform.rotation.x + tf_msg_v.transform.rotation.y * tf_msg_v.transform.rotation.z);
                // double cosr_cosp = 1 - 2 * (tf_msg_v.transform.rotation.x * tf_msg_v.transform.rotation.x + tf_msg_v.transform.rotation.y * tf_msg_v.transform.rotation.y);
                // double roll = std::atan2(sinr_cosp, cosr_cosp);

                // double sinp = 2 * (tf_msg_v.transform.rotation.w * tf_msg_v.transform.rotation.y - tf_msg_v.transform.rotation.z * tf_msg_v.transform.rotation.x);
                // double pitch = 0;
                // if (std::abs(sinp) >= 1)
                //     pitch = std::copysign(M_PI / 2, sinp);
                // else
                //     pitch = std::asin(sinp);

                // double siny_cosp = 2 * (tf_msg_v.transform.rotation.w * tf_msg_v.transform.rotation.z + tf_msg_v.transform.rotation.x * tf_msg_v.transform.rotation.y);
                // double cosy_cosp = 1 - 2 * (tf_msg_v.transform.rotation.y * tf_msg_v.transform.rotation.y + tf_msg_v.transform.rotation.z * tf_msg_v.transform.rotation.z);
                // double yaw = std::atan2(siny_cosp, cosy_cosp);
                
                tf2::Matrix3x3 m(q_);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
                double roll=0.0, pitch=0.0, yaw=0.0;
                m.getRPY(roll, pitch, yaw);

                std::string str_r = " ROTATION r : " + std::to_string(roll) + " p : " + std::to_string(pitch) + " y : " + std::to_string(yaw);

                value_text.write(str.c_str(), str.size());
                value_text.write(str_r.c_str(), str_r.size());
                value_text.close();
            }

            // broadcast tf_msg
            br.sendTransform(tf_msg_v);
            br.sendTransform(tf_msg_m);

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
