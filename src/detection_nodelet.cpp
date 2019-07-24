#include <robocross2019/DetectionConfig.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/camera_publisher.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Vector3Stamped.h>

class RedDeadDetectionNodelet : public nodelet::Nodelet
{
public:
	RedDeadDetectionNodelet() :
		camera_matrix(3, 3, CV_64F), distortion(8, 1, CV_64F)
	{}
private:
	image_transport::CameraSubscriber src_sub;
	image_transport::Publisher debug_threshold;
	image_transport::Publisher debug_center;

	ros::Publisher direction_pub;

	cv::Mat camera_matrix, distortion;

	boost::shared_ptr<dynamic_reconfigure::Server<red_dead_detection::DetectionConfig>> dyn_srv;
	red_dead_detection::DetectionConfig detect_cfg;

	void onInit()
	{
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& nh_priv = getPrivateNodeHandle();
		image_transport::ImageTransport it(nh);
		image_transport::ImageTransport it_priv(nh_priv);

		src_sub = it.subscribeCamera("image_raw", 1, &RedDeadDetectionNodelet::imageCallback, this);
		debug_threshold = it_priv.advertise("threshold", 1);
		debug_center = it_priv.advertise("debug", 1);
		direction_pub = nh_priv.advertise<geometry_msgs::Vector3Stamped>("direction", 1, false);

		dyn_srv = boost::make_shared<dynamic_reconfigure::Server<red_dead_detection::DetectionConfig>>(nh_priv);
		dynamic_reconfigure::Server<red_dead_detection::DetectionConfig>::CallbackType cb;

		cb = boost::bind(&RedDeadDetectionNodelet::paramCallback, this, _1, _2);
		dyn_srv->setCallback(cb);
	}

	void paramCallback(red_dead_detection::DetectionConfig &config, uint32_t level)
	{
		detect_cfg = config;
	}

	void storeCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo)
	{
		for(int i = 0; i < 3; ++i)
		{
			for(int j = 0; j < 3; ++j)
			{
				camera_matrix.at<double>(i, j) = cinfo->K[3 * i + j];
			}
		}
		for(int k = 0; k < cinfo->D.size(); k++)
		{
			distortion.at<double>(k) = cinfo->D[k];
		}
	}

	static geometry_msgs::Vector3 normalize(const geometry_msgs::Vector3 v)
	{
		double norm = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		geometry_msgs::Vector3 result;
		result.x = v.x / norm;
		result.y = v.y / norm;
		result.z = v.z / norm;
		return result;
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& src, const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		bool has_subscribers = (debug_threshold.getNumSubscribers() > 0) ||
				(debug_center.getNumSubscribers() > 0) ||
				(direction_pub.getNumSubscribers() > 0);
		if (!has_subscribers) return;

		storeCameraInfo(cameraInfo);
		auto src_image = cv_bridge::toCvShare(src);

		cv::Mat src_image_hsv;
		cv::cvtColor(src_image->image, src_image_hsv, cv::COLOR_BGR2HSV);

		cv::Mat img_threshold;
		cv::Scalar lower = cv::Scalar(detect_cfg.h_low, detect_cfg.s_low, detect_cfg.v_low);
		cv::Scalar upper = cv::Scalar(detect_cfg.h_high, detect_cfg.s_high, detect_cfg.v_high);
		cv::inRange(src_image_hsv, lower, upper, img_threshold);

		const cv::Size KERNEL_SIZE = {detect_cfg.kernel_size_x, detect_cfg.kernel_size_y};
		auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, KERNEL_SIZE);
		cv::erode(img_threshold, img_threshold, kernel);
		cv::dilate(img_threshold, img_threshold, kernel);

		auto moments = cv::moments(img_threshold, true);
		if (moments.m00 > 0)
		{
			cv::Point2d cnt{moments.m10 / moments.m00, moments.m01 / moments.m00};
			// m00 is the area of the pixels; radius would then be sqrt(area/pi)
			int approx_radius = std::sqrt(moments.m00 * M_1_PI);

			// Calculate direction vector
			std::vector<cv::Point2d> cnt_distort = { cnt };
			std::vector<cv::Point2d> cnt_undistort(1);
			cv::undistortPoints(cnt_distort, cnt_undistort, camera_matrix, distortion, cv::noArray(), camera_matrix);
			// Shift points to center
			cnt_undistort[0].x -= src->width / 2;
			cnt_undistort[0].y -= src->height / 2;

			double fx = camera_matrix.at<double>(0, 0);
			double fy = camera_matrix.at<double>(1, 1);
			geometry_msgs::Vector3Stamped direction;
			direction.vector.x = cnt.x - src->width / 2; //cnt_undistort[0].x;
			// Reverse Y direction due to screen coordinates being upside down
			direction.vector.y = cnt.y - src->height / 2 + 50 /* HACK! Remove later*/; //cnt_undistort[0].y * fx / fy;
			direction.vector.z = src->width / 2;
			// Normalize vector
			direction.vector = normalize(direction.vector);
			direction.header.stamp = src->header.stamp;
			direction.header.frame_id = src->header.frame_id;
			//ROS_INFO_THROTTLE(1, "Publishing vector: (%lf, %lf, %lf)", direction.vector.x, direction.vector.y, direction.vector.z);
			direction_pub.publish(direction);
			cnt_undistort[0].x += src->width / 2;
			cnt_undistort[0].y += src->height / 2;

			if (debug_center.getNumSubscribers() > 0)
			{
				cv_bridge::CvImage debug_msg;
				debug_msg.header.frame_id = src->header.frame_id;
				debug_msg.header.stamp = src->header.stamp;
				debug_msg.encoding = sensor_msgs::image_encodings::BGR8;
				cv::Mat debug_image = src_image->image;
				cv::circle(debug_image, cnt, approx_radius, cv::Scalar(0, 255, 0), 3);
				cv::circle(debug_image, cnt_undistort[0], approx_radius, cv::Scalar(255, 0, 0), 3);
				debug_msg.image = debug_image;
				debug_center.publish(debug_msg.toImageMsg());
			}
		}

		if (debug_threshold.getNumSubscribers() > 0)
		{
			cv_bridge::CvImage debug_msg;
			debug_msg.header.frame_id = src->header.frame_id;
			debug_msg.header.stamp = src->header.stamp;
			debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
			debug_msg.image = img_threshold;
			debug_threshold.publish(debug_msg.toImageMsg());
		}
	}
};

PLUGINLIB_EXPORT_CLASS(RedDeadDetectionNodelet, nodelet::Nodelet)
