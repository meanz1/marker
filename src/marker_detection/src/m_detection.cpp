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

#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

cv::Mat frame;
bool flag = false;

std::mutex pcl_cloud_mtx;
pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
pcl::visualization::PCLVisualizer PCL_viewer("Cloud Viewer");

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

void imagecallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // ROS_INFO("Size : %d %d", msg->width, msg->height);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        frame = cv_ptr->image;
        if (frame.empty())
            ROS_ERROR("frame empty");

        flag = true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void PCLcallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	try
	{
		pcl::PointCloud<pcl::PointXYZRGB> temp_pcl_cloud;
		pcl::fromROSMsg(*msg, temp_pcl_cloud);

		pcl_cloud_mtx.lock();
		pcl_cloud.swap(temp_pcl_cloud);
		pcl_cloud_mtx.unlock();
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
}

void getMats(std::vector<std::vector<cv::Point3f>>& board_, std::vector<std::vector<cv::Point2f>>& corners_, std::vector<int>& ids_, cv::Mat& ref, cv::Mat& ob)
{
	pcl_cloud_mtx.lock();
	pcl::PointCloud<pcl::PointXYZRGB> temp_pcl_cloud = pcl_cloud;
	pcl_cloud_mtx.unlock();

	for(int i=0; i<(int)corners_.size(); i++)
		for(int j=0; j<(int)corners_[i].size(); j++)
		{
			pcl::PointXYZRGB p = temp_pcl_cloud.at(corners_[i][j].x, corners_[i][j].y);
			int corner_row = i/7;
			int corner_col = i%7;
			int board_idx = ids_[i];

			if(isnan(p.x) || isnan(p.y) || isnan(p.z))
			{
				continue;
			}

			cv::Mat ob_point(1, 3, CV_64FC1, cv::Scalar(0));
			ob_point.at<double>(0, 0) = p.x;
			ob_point.at<double>(0, 1) = p.y;
			ob_point.at<double>(0, 2) = p.z;
			if(ob.empty()) ob = ob_point.clone();
			else ob.push_back(ob_point.clone());

			cv::Mat ref_point(1, 3, CV_64FC1, cv::Scalar(0));
			ref_point.at<double>(0, 0) = board_[board_idx][j].x;
			ref_point.at<double>(0, 1) = board_[board_idx][j].y;
			ref_point.at<double>(0, 2) = board_[board_idx][j].z;
			if(ref.empty()) ref = ref_point.clone();
			else ref.push_back(ref_point.clone());
		}
}

cv::Mat getZeroMean(cv::Mat& input, cv::Mat& mean)
{
	int size = input.rows;
	int dimension = input.cols;

	cv::Mat output = input.clone();

	mean = cv::Mat(1, dimension, input.type(), cv::Scalar(0));
	for(int i=0; i<size; i++)
		mean += input.row(i);

	mean /= (double)size;

	for(int i=0; i<dimension; i++)
		output.col(i) -= mean.at<double>(0, i);

	return output;
}

void calcPose(cv::Mat& ref, cv::Mat& ob, cv::Mat& R_, cv::Mat& T_)
{
	if(ref.size() != ob.size())
	{
		ROS_ERROR("ref and ob size are not same");
		return;
	}

	cv::Mat output;

	//zero mean
	cv::Mat ref_mean, ob_mean;
	cv::Mat ref_zeroMean = getZeroMean(ref, ref_mean);
	cv::Mat ob_zeroMean = getZeroMean(ob, ob_mean);

	//calc pose
	cv::Mat M = ob_zeroMean.t() * ref_zeroMean;
	cv::SVD svd(M);
	R_ = svd.vt.t() * svd.u.t();

	if(cv::determinant(R_) < 0)
	{
		double data[9] {1, 0, 0, 0, 1, 0, 0, 0, -1};
		R_ = svd.vt.t() * cv::Mat(3, 3, CV_64FC1, data) * svd.u.t();
	}

	T_ = ref_mean.t() - R_ * ob_mean.t();
}

void pub_PointTF(std::vector<std::vector<cv::Point3f>>& target, std::string frame, std::string prefix, tf::TransformBroadcaster& br_)
{
	double r_ = 0, p_ = 0, y_ = 0;
	for(int i=0; i<(int)target.size(); i++)
	{
		for(int j=0; j<(int)target[i].size(); j++)
		{
			tf::StampedTransform cam2aruco = create_stamped_transform(
				target[i][j].x, target[i][j].y, target[i][j].z, 
				r_, p_, y_, 
				frame, cv::format("%s_%02d_%02d", prefix.c_str(), i, j)
			);

			br_.sendTransform(cam2aruco);
		}
	}
}

void pub_PointTF(cv::Mat& target, std::string frame, std::string prefix, tf::TransformBroadcaster& br_)
{
	double r_ = 0, p_ = 0, y_ = 0;

	if(target.cols == 3)
	{
		for(int i=0; i<(int)target.rows; i++)
		{
			tf::StampedTransform cam2aruco = create_stamped_transform(
				target.at<double>(i, 0), target.at<double>(i, 1), target.at<double>(i, 2), 
				r_, p_, y_, 
				frame, cv::format("%s_%03d", prefix.c_str(), i)
			);

			br_.sendTransform(cam2aruco);
		}
	}
	else if(target.rows == 3)
	{
		for(int i=0; i<(int)target.cols; i++)
		{
			tf::StampedTransform cam2aruco = create_stamped_transform(
				target.at<double>(0, i), target.at<double>(1, i), target.at<double>(2, i), 
				r_, p_, y_, 
				frame, cv::format("%s_%d", prefix.c_str(), i)
			);

			br_.sendTransform(cam2aruco);
		}
	}
	else ROS_WARN("Unknown target type");
}

std::vector<std::vector<cv::Point3f>> transform(std::vector<std::vector<cv::Point3f>>& target, tf::StampedTransform& tf)
{
	tf::Matrix3x3 tf_R(tf.getRotation());
	tf::Vector3 tf_T(tf.getOrigin());
	std::vector<std::vector<cv::Point3f>> output;
	for(int i=0; i<(int)target.size(); i++)
	{
		output.push_back(std::vector<cv::Point3f>());
		for(int j=0; j<(int)target[i].size(); j++)
		{
			tf::Vector3 p(target[i][j].x, target[i][j].y, target[i][j].z);
			tf::Vector3 tf_p = tf_R*p + tf_T;
			output[i].push_back(cv::Point3f(tf_p.getX(), tf_p.getY(), tf_p.getZ()));
		}
	}

	return output;
}
std::vector<std::vector<cv::Point3f>> transform(std::vector<std::vector<cv::Point3f>>& target, std::string from, std::string to, tf::TransformListener& ls_)
{
	tf::StampedTransform tf;
	try{ ls_.lookupTransform(from, to, ros::Time(0), tf); }
	catch(...) { return std::vector<std::vector<cv::Point3f>>(); }
	return transform(target, tf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

	PCL_viewer.addCoordinateSystem(0.1);

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.148, 0.015, dictionary);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.0417, 0.0056, dictionary);

	// tf::StampedTransform base2marker = create_stamped_transform(0.77, -0.505, 0, 0, 0, 0, "base_link", "marker");
	// tf::StampedTransform base2marker = create_stamped_transform(0.6, 0, -0.01, 0, -M_PI/2.0, 0, "base_link", "marker");
	// tf::StampedTransform base2marker = create_stamped_transform(0.482, -0.144, 0.77, 0, -0.663, 0, "base_link", "marker");
	tf::StampedTransform base2marker = create_stamped_transform(0.85, -0.31, 0.675, 0, -0.95, 0, "base_link", "marker");

	std::vector<std::vector<cv::Point3f>> tf_boardPoints = transform(board->objPoints, base2marker);
	
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_image = it.subscribe("/camera/rgb/image_rect_color", 1, imagecallback);
	ros::Subscriber sub_points = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &PCLcallback);

	std::string package_path = ros::package::getPath("marker_detection");
    std::string filename = package_path + "/src/cam_int";

    std::deque<std::pair<cv::Point3d, cv::Point3d>> dq_xyzrpy;

    std::vector<int> ids;

    cv::Mat camMatrix, distCoeffs;

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

    tf::StampedTransform tf_b2c;
    tf::TransformListener listener;
    tf::TransformListener listener_cam_link;

    tf::TransformListener listener_cam;
    ros::Rate loop_rate(10);

    tf::Matrix3x3 m2c_r;
    tf::Vector3 m2c_t;

    while (ros::ok())
    {
		PCL_viewer.removeAllPointClouds();
		PCL_viewer.removeCorrespondences();

		base2marker.stamp_ = ros::Time::now();
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
		std::vector<std::vector<cv::Point2f>> corners;
		cv::aruco::detectMarkers(frame, dictionary, corners, ids);

		if(ids.size() <= 0 || pcl_cloud.empty())
		{
			if(ids.size() <= 0) ROS_ERROR("Fail to detect Markers");
			else if(pcl_cloud.empty()) ROS_ERROR("Empty pcl_cloud");

			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}

		if (ids.size() > 0)
		{
			cv::Mat ob_points, ref_points;
			getMats(tf_boardPoints, corners, ids, ref_points, ob_points); //N x 3 data

			pub_PointTF(ref_points, "base_link", "ref", br_1);
			pub_PointTF(ob_points, "camera_rgb_optical_frame", "ob", br_1);

			cv::Mat R, tvec_depth, rvec_depth;
			calcPose(ref_points, ob_points, R, tvec_depth); //ref_points(base_link) ob_poitns(cam)
			cv::Rodrigues(R, rvec_depth);

			tf::StampedTransform result_tf = create_stamped_transform(tvec_depth, rvec_depth, "base_link", "cam");
			
			if(!rvec_depth.empty() && !tvec_depth.empty())
				br_1.sendTransform(result_tf);

			if(1)
			{
				cv::Mat result = R*ob_points.t();
				result.row(0) += tvec_depth.at<double>(0, 0);
				result.row(1) += tvec_depth.at<double>(0, 1);
				result.row(2) += tvec_depth.at<double>(0, 2);
				cv::Mat result_t = result.t();

				pcl_cloud_mtx.lock();
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_pcl_cloud = pcl_cloud.makeShared();
				pcl_cloud_mtx.unlock();

				PCL_viewer.addPointCloud(temp_pcl_cloud);

				pcl::PointCloud<pcl::PointXYZ>::Ptr ob_PCL(new pcl::PointCloud<pcl::PointXYZ>());
				for(int i=0; i<ob_points.rows; i++)
					ob_PCL->push_back(pcl::PointXYZ(ob_points.at<double>(i, 0), ob_points.at<double>(i, 1), ob_points.at<double>(i, 2)));
				PCL_viewer.addPointCloud(ob_PCL, "ob");
				PCL_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "ob");
				PCL_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 5, "ob");

				pcl::PointCloud<pcl::PointXYZ>::Ptr ref_PCL(new pcl::PointCloud<pcl::PointXYZ>());
				for(int i=0; i<ref_points.rows; i++)
					ref_PCL->push_back(pcl::PointXYZ(ref_points.at<double>(i, 0), ref_points.at<double>(i, 1), ref_points.at<double>(i, 2)));
				PCL_viewer.addPointCloud(ref_PCL, "ref");
				PCL_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "ref");
				PCL_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 5, "ref");

				pcl::PointCloud<pcl::PointXYZ>::Ptr result_PCL(new pcl::PointCloud<pcl::PointXYZ>());
				for(int i=0; i<result_t.rows; i++)
					result_PCL->push_back(pcl::PointXYZ(result_t.at<double>(i, 0), result_t.at<double>(i, 1), result_t.at<double>(i, 2)));
				PCL_viewer.addPointCloud(result_PCL, "result");
				PCL_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "result");
				PCL_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 5, "result");

				std::vector<int> corrs;
				for(int i=0; i<(int)ob_PCL->size(); i++)
					corrs.push_back(i);
				PCL_viewer.addCorrespondences<pcl::PointXYZ>(result_PCL, ref_PCL, corrs);	
			}

			if(0)
			{
				for(int i=0; i<(int)board->objPoints.size(); i++)
				{
					std::vector<cv::Point2f> projP;

//					cv::projectPoints(board->objPoints[i], rvec_board, tvec_board, camMatrix, distCoeffs, projP);
//					for(int j=0; j<(int)projP.size(); j++)
//						cv::circle(frame_cp, projP[j], 2, cv::Scalar(0, 0, 255));
					cv::projectPoints(tf_boardPoints[i], cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camMatrix, distCoeffs, projP);
					for(int j=0; j<(int)projP.size(); j++)
						cv::circle(frame_cp, projP[j], 2, cv::Scalar(0, 0, 255));
				}
			}
		}

		if(0 && listener_cam_link.waitForTransform("/camera_rgb_optical_frame", "/camera_link", ros::Time(0), ros::Duration(1.0)))
		{
			tf::StampedTransform tf_o2l;
			listener_cam_link.lookupTransform("/camera_rgb_optical_frame", "/camera_link", ros::Time(0), tf_o2l);

			tf::StampedTransform base_st(tf::Transform(tf_o2l.getRotation(), tf_o2l.getOrigin()), ros::Time::now(), "cam", "est_camera_link");
			br_1.sendTransform(base_st);

				tf::Matrix3x3 m(base_st.getRotation());
				double roll = 0.0, pitch = 0.0, yaw = 0.0;
				m.getRPY(roll, pitch, yaw);

				ROS_WARN("%lf %lf %lf, %lf %lf %lf", base_st.getOrigin().getX(), base_st.getOrigin().getY(), base_st.getOrigin().getZ(), roll, pitch, yaw);
		}
		else br_1.sendTransform(create_stamped_transform(-0.045, 0.0, 0.0, 1.570796, -1.570796, 0.000000, "cam", "est_camera_link"));

		if(0)
		{
			std::string from = "/est_camera_link", to = "/marker";
			// std::string from = "/est_camera_link", to = "/camera_link";
			if(listener.waitForTransform(from, to, ros::Time(0), ros::Duration(1.0)))
			{
				tf::StampedTransform tf;
				listener.lookupTransform(from, to, ros::Time(0), tf);

				tf::Matrix3x3 m(tf.getRotation());
				double roll = 0.0, pitch = 0.0, yaw = 0.0;
				m.getRPY(roll, pitch, yaw);

				ROS_WARN("%lf %lf %lf", roll, pitch, yaw);
			}
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
		}

		cv::Mat resized_frame_cp;
		cv::resize(frame_cp, resized_frame_cp, cv::Size(frame_cp.cols*2, frame_cp.rows*2));
		cv::imshow("out", resized_frame_cp);
		if (cv::waitKey(1) == 27)
			break;

        ros::spinOnce();
		PCL_viewer.spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
