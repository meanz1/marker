#include <iostream>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "ros/package.h"
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

bool cout_flag = false;

cv::Mat frame;
bool flag = false;
tf::Quaternion q_;
// marker2cam (matrix)
tf::Matrix3x3 R_cam_link;
tf::Vector3 T_cam_link;

tf::Matrix3x3 R_cam_link2;
tf::Vector3 T_cam_link2;

// adverse
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
	tf::Quaternion q;

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

tf::Transform create_transform(const cv::Mat &tvec, const cv::Mat &rvec)
{
    tf::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3_m(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion_m(rvec));
    return transform;
}

tf::StampedTransform create_stamped_transform(const cv::Mat &tvec, const cv::Mat &rvec, std::string frame_id, std::string child_frame_id)
{
    tf::StampedTransform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3_m(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion_m(rvec));

	transform.stamp_ = ros::Time::now();
	transform.frame_id_ = frame_id;
	transform.child_frame_id_ = child_frame_id;
	
    return transform;
}

tf::StampedTransform create_stamped_transform(double x, double y, double z, double r, double p, double yaw, 
		std::string frame_id, std::string child_frame_id)
{
	tf::Quaternion q;
	q.setRPY(r, p, yaw);

    tf::StampedTransform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(q);

	transform.stamp_ = ros::Time::now();
	transform.frame_id_ = frame_id;
	transform.child_frame_id_ = child_frame_id;
	
    return transform;
}
void imagecallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // ROS_INFO("Size : %d %d", msg->width, msg->height);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame = cv_ptr->image;
        if (frame.empty())
            std::cout << "frame empty" << std::endl;

        flag = true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
int main(int argc, char **argv)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.148, 0.015, dictionary);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.0417, 0.0056, dictionary);

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

	std::string package_path = ros::package::getPath("marker_detection");
    std::string filename = package_path + "/src/cam_int";

    // ros::Publisher tf_list_pub_ = nh.advertise<tf_msgs::TFMessage>("/tf", 10);
    // std::deque<double> dqx;
    // std::deque<double> dqy;
    // std::deque<double> dqz;
    std::deque<std::pair<cv::Point3d, cv::Point3d>> dq_xyzrpy;

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

    tf::TransformBroadcaster br_1;
    tf::TransformBroadcaster br_2;
    tf::TransformBroadcaster br_3;
    tf::TransformBroadcaster br_4;
    tf::TransformBroadcaster br_5;
    // tf_msgs::TFMessage tf_msg_list_;
    tf::TransformListener listener_base2camera_link;
    cv::FileStorage f_s(filename, cv::FileStorage::READ);

    if (!f_s.isOpened())
        std::cout << "err" << std::endl;

    f_s["camera_matrix"] >> camMatrix;
    f_s["distortion_coefficients"] >> distCoeffs;

    f_s.release();

    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imagecallback);
    tf::StampedTransform tf_b2c;
    tf::TransformListener listener;
    tf::TransformListener listener_cam_link;

    tf::TransformListener listener_cam;
    ros::Rate loop_rate(10);

    tf::Matrix3x3 m2c_r;
    tf::Vector3 m2c_t;

    while (ros::ok())
    {
//		tf::StampedTransform base2marker = create_stamped_transform(0.77, -0.505, 0, 0, 0, 0, "base_link", "marker");
		tf::StampedTransform base2marker = create_stamped_transform(0.6, 0, -0.01, 0, -M_PI/2.0, 0, "base_link", "marker");
		br_1.sendTransform(base2marker);

		if(frame.empty() || flag == false)
		{
			ROS_ERROR("frame is empty");
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}

		cv::Mat frame_cp;
        frame.copyTo(frame_cp);

		bool find_marker = false;
		cv::aruco::detectMarkers(frame, dictionary, corners, ids);

		if(ids.size() <= 0)
		{
			ROS_ERROR("Fail to detect Markers");
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}

		if (ids.size() > 0)
		{
			cv::aruco::drawDetectedMarkers(frame_cp, corners, ids);

			int find_marker = cv::aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec_board, tvec_board);

			if (find_marker > 0)
			{
				cv::aruco::drawAxis(frame_cp, camMatrix, distCoeffs, rvec_board, tvec_board, 0.1);
			}
		}

		tf::Transform cam2marker = create_transform(cv::Mat(tvec_board), cv::Mat(rvec_board));
		tf::Transform marker2cam = cam2marker.inverse();

		// tf::StampedTransform tf_marker2cam(marker2cam, ros::Time::now(), "marker", "cam");
		tf::StampedTransform tf_marker2cam(marker2cam, ros::Time::now(), "marker", "cam");
		br_1.sendTransform(tf_marker2cam);

		// try
		// {
		// 	bool scs = listener.waitForTransform("/base_link", "/cam", ros::Time(0), ros::Duration(1.0));
		// 	if (scs) listener.lookupTransform("/base_link", "/cam", ros::Time(0), tf_m);
		// }
		// catch(...) 
		// {
		// 	ROS_WARN("Cannot get tf /base_link -> /cam");
		// 	ros::spinOnce();
		// 	loop_rate.sleep();
		// 	continue;
		// }

		// cv::Mat R_inv, T_inv;
		// cv::Rodrigues(rvec_board, R);
		// R_inv = R.t();
		// cv::Rodrigues(R_inv, rvecs_inv);
		// T_inv = -R_inv * tvec_board;
		// cv::Rodrigues(tvec_board, T_inv);
		// T_inv = T_inv.inv();

		// std::cout << tvec_board[0] << " " << tvec_board[1] << " " << tvec_board[2] << " " <<std::endl;
		// std::cout << T_inv.at<double>(0,0) << " " << T_inv.at<double>(1,0) << " " << T_inv.at<double>(2,0) << " " <<std::endl;

		// tf::Transform transform_m = create_transform(T_inv, rvecs_inv);
		// marker2cam
		// tf::StampedTransform tf_m;
		// auto transform_m = create_transform_m(T_inv, rvecs_inv); // m2c
		// m2c_t[0] = T_inv.at<double>(0, 0);
		// m2c_t[1] = T_inv.at<double>(1, 0);
		// m2c_t[2] = T_inv.at<double>(2, 0);

		// std::cout << " R_inv.size() : " << rvecs_inv.rows << " " << rvecs_inv.cols << std::endl;

		// std::cout << m2c_t[0] << " " << m2c_t[1] << " " << m2c_t[2] << " " << std::endl;

		// m2c_r[0][0] = R_inv.at<double>(0, 0);
		// m2c_r[0][1] = R_inv.at<double>(0, 1);
		// m2c_r[0][2] = R_inv.at<double>(0, 2);
		// m2c_r[1][0] = R_inv.at<double>(1, 0);
		// m2c_r[1][1] = R_inv.at<double>(1, 1);
		// m2c_r[1][2] = R_inv.at<double>(1, 2);
		// m2c_r[2][0] = R_inv.at<double>(2, 0);
		// m2c_r[2][1] = R_inv.at<double>(2, 1);
		// m2c_r[2][2] = R_inv.at<double>(2, 2);


		// tf::Transform transform_m(m2c_r, m2c_t);
		// tf::Transform tf_msg_m;
		// tf_msg_m.setOrigin(tf::Vector3(transform_m.getOrigin().getX(), transform_m.getOrigin().getY(), transform_m.getOrigin().getZ()));
		// tf_msg_m.setRotation(tf::Quaternion(transform_m.getRotation().getX(), transform_m.getRotation().getY(), transform_m.getRotation().getZ(), transform_m.getRotation().getW()));
		// tf::StampedTransform tf_st_msg_m(transform_m, ros::Time::now(), "marker", "cam");
		// br_1.sendTransform(tf_st_msg_m);

		// tf::Transform ttt;
		// ttt.setOrigin(tf::Vector3(tf_m.getOrigin().getX(), tf_m.getOrigin().getY(), tf_m.getOrigin().getZ()));
		// ttt.setRotation(tf::Quaternion(tf_m.getRotation().getX(), tf_m.getRotation().getY(), tf_m.getRotation().getZ(), tf_m.getRotation().getW()));
		// tf::StampedTransform tf_st_msg_bc(ttt, ros::Time::now(), "base_link", "t");
		// br_1.sendTransform(tf_st_msg_bc);

		// ---------------- clear ------------------
		// ---------------- clear ------------------

		tf::StampedTransform tf_cam_msg2;
		tf::StampedTransform tf_base2camera_link;

		// edit /////////////now /////////////////


		if(listener_cam_link.waitForTransform("/camera_rgb_optical_frame", "/camera_link", ros::Time(0), ros::Duration(1.0)))
		{
			tf::StampedTransform tf_o2l;
			listener_cam_link.lookupTransform("/camera_rgb_optical_frame", "/camera_link", ros::Time(0), tf_o2l);

			tf::StampedTransform base_st(tf::Transform(tf_o2l.getRotation(), tf_o2l.getOrigin()), ros::Time::now(), "cam", "est_camera_link");
			br_1.sendTransform(base_st);
		}
			
	   

		if(listener.waitForTransform("/base_link", "/est_camera_link", ros::Time(0), ros::Duration(1.0)))
		{
			tf::StampedTransform tf_b2d;
			listener.lookupTransform("/base_link", "/est_camera_link", ros::Time(0), tf_b2d);

			tf::Matrix3x3 m(tf_b2d.getRotation());
			double roll = 0.0, pitch = 0.0, yaw = 0.0;
			m.getRPY(roll, pitch, yaw);

			if (dq_xyzrpy.size() >= 10)
				dq_xyzrpy.pop_front();

			dq_xyzrpy.push_back(std::pair<cv::Point3d, cv::Point3d>(
				cv::Point3d(tf_b2d.getOrigin().getX(), tf_b2d.getOrigin().getY(), tf_b2d.getOrigin().getZ()),
				cv::Point3d(roll, pitch, yaw)
			));

			if (dq_xyzrpy.size() == 10)
			{
				std::ofstream value_text;
				value_text.open(package_path + "/src/value.txt");

				cv::Point3d t_sum(0, 0, 0);
				cv::Point3d t_ave(0, 0, 0);
				cv::Point3d r_sum(0, 0, 0);
				cv::Point3d r_ave(0, 0, 0);

				for (int i = 0; i < dq_xyzrpy.size(); i++)
				{
					t_sum += dq_xyzrpy[i].first;
					r_sum += dq_xyzrpy[i].second;
				}

				t_ave = t_sum / (double)dq_xyzrpy.size();
				r_ave = r_sum / (double)dq_xyzrpy.size();

				std::string str = "TRANSFORMATION x : " + std::to_string(t_ave.x) + " y : " + std::to_string(t_ave.y) + " z : " + std::to_string(t_ave.z);
				std::string str_r = " ROTATION r : " + std::to_string(r_ave.x) + " p : " + std::to_string(r_ave.y) + " y : " + std::to_string(r_ave.z);

				value_text.write(str.c_str(), str.size());
				value_text.write(str_r.c_str(), str_r.size());
				value_text.close();

				ROS_INFO("%lf %lf %lf %lf %lf %lf", t_ave.x, t_ave.y, t_ave.z, r_ave.x, r_ave.y, r_ave.z);
			}


			if(1)
			{
				cv::Mat R;
				cv::Rodrigues(rvec_board, R);
				
				for(int i=0; i<(int)board->objPoints.size(); i++)
				{
					for(int j=0; j<(int)board->objPoints[i].size(); j++)
					{
						cv::Mat p(3, 1, CV_64FC1, cv::Scalar(0));
						p.at<double>(0, 0) = board->objPoints[i][j].x;
						p.at<double>(1, 0) = board->objPoints[i][j].y;
						p.at<double>(2, 0) = board->objPoints[i][j].z;

						cv::Mat proj_p = R*p;

						proj_p += tvec_board;

						// tf::Matrix3x3 tf_m(cv_vector3d_to_tf_quaternion_m(rvecs_inv));
						double r_ = 0, p_ = 0, y_ = 0;
						// tf_m.getRPY(r_, p_, y_);

						tf::StampedTransform cam2aruco = create_stamped_transform(
							proj_p.at<double>(0, 0), proj_p.at<double>(1, 0), proj_p.at<double>(2, 0), 
							r_, p_, y_, 
							"cam", cv::format("%d_%d", i, j));

						br_1.sendTransform(cam2aruco);
					}
				}
			}
		}

		cv::Mat resized_frame_cp;
		cv::resize(frame_cp, resized_frame_cp, cv::Size(frame_cp.cols*2, frame_cp.rows*2));
		cv::imshow("out", resized_frame_cp);
		if (cv::waitKey(1) == 27)
			break;

		if(cout_flag)
		{
			for (int i = 0; i < ids.size(); i++)
			{
				for (int j = 0; j < 4; j++)
				{
					std::cout << "[" << i << "] : " << corners[i][j] << std::endl;
				}
			}
		}

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
