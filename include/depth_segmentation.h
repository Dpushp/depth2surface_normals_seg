#ifndef DEPTH_SEGMENTATION_H
#define DEPTH_SEGMENTATION_H
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <depth2surface_normals_seg/DenoiseConfig.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/tf.h>

#include <cmath>
#include <string>

class DepthSegmentation
{
private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher normal_pub_;
    image_transport::Publisher segmented_ground_pub_;
public:
    DepthSegmentation(ros::NodeHandle nh_);
    ~DepthSegmentation();

    /* Callback */
    void realsense2ImageCallback(const sensor_msgs::ImageConstPtr& msg);

    /* Utility */
    cv::Mat trim(cv::Mat depth);
    cv::Mat denoise(cv::Mat depth);
    cv::Mat colorized_depth(cv::Mat depth);
    cv::Mat surface_normals(cv::Mat depth);
    cv::Mat find_Ir(cv::Mat normals);

    // Frame id to publish image topic
    std::string frame_id;
    bool is_realsense;

    void updateConfig(int d_erosion_size, int d_gaussian_blur_size, int seg_erosion_size, int iLowH, int iLowS, int iLowV, int iHighH, int iHighS, int iHighV);
};
#endif // DEPTH_SEGMENTATION_H

