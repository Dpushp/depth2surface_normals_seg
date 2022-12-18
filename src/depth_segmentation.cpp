#include<depth_segmentation.h>

// Params
// Depth denoising 
int _d_erosion_size = 2;
int _d_gaussian_blur_size = 11;

// Segmentation
int _seg_erosion_size = 5; 
int _iLowH = 30;
int _iLowS = 150;
int _iLowV = 60; 
int _iHighH = 110; 
int _iHighS = 255; 
int _iHighV = 255;


DepthSegmentation::DepthSegmentation(ros::NodeHandle nh_)
: it_(nh_)
{
    // Read the topics from the launch file params
    std::string depth_image_topic;
    std::string surface_normals_image_topic;
    std::string ground_segenmation_image_topic;
    std::string frame_id;
    bool is_realsense;
    
    if (nh_.getParam("/sim_ground_segmentation/depth_image_topic", depth_image_topic) &&
        nh_.getParam("/sim_ground_segmentation/surface_normals_image_topic", surface_normals_image_topic) &&
        nh_.getParam("/sim_ground_segmentation/ground_segenmation_image_topic", ground_segenmation_image_topic) &&
        nh_.getParam("/sim_ground_segmentation/frame_id", frame_id) &&
        nh_.getParam("/sim_ground_segmentation/is_realsense", is_realsense))
    {
        this->frame_id = frame_id;
        this->is_realsense = is_realsense;
        ROS_INFO("Received all params!");
    }
    else
    {
        ROS_ERROR("Failed to get param 'depth_image_topic'");
    }

    // Subscribe images from realsense camera
    image_sub_ = it_.subscribe(depth_image_topic, 1, &DepthSegmentation::realsense2ImageCallback, this);
    normal_pub_ = it_.advertise(surface_normals_image_topic, 1);
    segmented_ground_pub_ = it_.advertise(ground_segenmation_image_topic, 1);		
}

DepthSegmentation::~DepthSegmentation()
{
}

/*! DyConfig function */
void DepthSegmentation::updateConfig(int d_erosion_size, int d_gaussian_blur_size, int seg_erosion_size, int iLowH, int iLowS, int iLowV, int iHighH, int iHighS, int iHighV){
    _d_erosion_size = d_erosion_size;
    _d_gaussian_blur_size = d_gaussian_blur_size;
    _seg_erosion_size = seg_erosion_size; 
    _iLowH = iLowH;
    _iLowS = iLowS;
    _iLowV = iLowV; 
    _iHighH = iHighH; 
    _iHighS = iHighS; 
    _iHighV = iHighV;
}

/*! Callback function */
void DepthSegmentation::realsense2ImageCallback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        // Read depth images from Turtlebot3 gazebo
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    /*! Copy the depth image. */
	cv::Mat depth = cv_ptr->image.clone();

    //  For turtlebot sim
    if(!is_realsense)
    {
        int row_truncate = 850;
        depth = depth.rowRange(0, row_truncate);
    
    }
    
    /*! Depth Image preprocessing */
    for( size_t i = 0; i < msg->height; i++ ){
        for( size_t j = 0; j < msg->width; j++ ){
            depth.at<float>(i, j) = 1000*depth.at<float>(i, j);
        }
    }
    
    // Surface Normal : dtype = CV_64FC3
    cv::Mat normals = surface_normals(depth.clone());
    
    // Convert CV_64FC3 to CV_8UC3
    cv::Mat nor_8UC3;
    normals.convertTo(nor_8UC3, CV_8UC3, 100);
    
    // Find Ir(Reachable states) i.e., Nav space for ground robot
    cv::Mat Ir = find_Ir(nor_8UC3);	

    /* Publish data for visualization */
    std_msgs::Header header;
    header.frame_id = frame_id;
    sensor_msgs::ImagePtr nor_msg = cv_bridge::CvImage(header, "bgr8", nor_8UC3).toImageMsg();
    normal_pub_.publish(nor_msg);

    sensor_msgs::ImagePtr sg_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", Ir).toImageMsg();
    segmented_ground_pub_.publish(sg_msg);
}

/*! Trim */
cv::Mat DepthSegmentation::trim(cv::Mat depth){
    /*! Intel RealSense D400 and SR300 series depth cameras output depth with 16-bit precision. */
    /*! Depth image colorization */
    /*! The proposed colorization process imposes the limitation of needing to fit a 16-bit depth map into a 10.5-bit color image. */
    /*! It is recommended to perform the colorization only after first limiting the depth range to a subset of the full depth 0 ~ 65535 range, and re-normalizing it. */

    /*! Threshold - removes values outside recommended range. */
    float min_depth = 0.1f;
    float max_depth = 10.0f;

    for(size_t i = 0; i < depth.rows; i++ ){
        for(size_t j = 0; j < depth.cols; j++ ){
            if((1.0*(depth.at<ushort>(i, j))/1000) > max_depth){
                depth.at<ushort>(i, j) = max_depth*1000;
            }
            if((1.0*(depth.at<ushort>(i, j))/1000) < min_depth){
                depth.at<ushort>(i, j) = min_depth*1000;
            }
        }
    }
    return depth;
}

/*! Denoise */
cv::Mat DepthSegmentation::denoise(cv::Mat depth){
    int erosion_type = 0;
    
    cv::Mat element = cv::getStructuringElement( erosion_type, cv::Size( 2*_d_erosion_size + 1, 2*_d_erosion_size+1 ), cv::Point( _d_erosion_size, _d_erosion_size ) );
    
    cv::dilate( depth, depth, element );
    cv::dilate( depth, depth, element );
    cv::dilate( depth, depth, element );
    cv::dilate( depth, depth, element );
    cv::dilate( depth, depth, element );
    //cv::erode( depth, depth, element );

    cv::GaussianBlur( depth, depth, cv::Size(_d_gaussian_blur_size, _d_gaussian_blur_size ), 0, 0 );
    cv::erode( depth, depth, element );
    
    return depth;
}

/* Get Colorized depth */
cv::Mat DepthSegmentation::colorized_depth(cv::Mat depth){
	cv::Mat depth_8UC, colored_depth;
	// Convert CV_16UC1 to CV_8UC1
	depth.convertTo(depth_8UC, CV_8UC1, 0.1);
	// Apply color Map
	cv::applyColorMap(depth_8UC, colored_depth, cv::COLORMAP_JET);
	return colored_depth;
}

/* Get Surface Normals (dtype: CV_64FC3) from depth image */
cv::Mat DepthSegmentation::surface_normals(cv::Mat depth){
	depth.convertTo(depth, CV_64FC1);
	cv::Mat nor(depth.size(), CV_64FC3);

	for(int x = 1; x < depth.cols - 1; ++x){
		for(int y = 1; y < depth.rows - 1; ++y){
			/* Top : depth(y-1,x) */
			cv::Vec3d t(x,y-1,depth.at<double>(y-1, x));
			/* Left : depth(y,x-1) */
			cv::Vec3d l(x-1,y,depth.at<double>(y, x-1));
			/* Center : depth(y,x) */
			cv::Vec3d c(x,y,depth.at<double>(y, x));
			
			// Cross product of two lines on a plane 
			cv::Vec3d d = (l-c).cross(t-c);
			// Normalize to get direction vector
			cv::Vec3d n = cv::normalize(d);
			
			nor.at<cv::Vec3d>(y,x) = n;
		}
	}
	return nor;
}

/*! Find Ir (Reachable states - pixels) from surface normal image. */
cv::Mat DepthSegmentation::find_Ir(cv::Mat normals){
    //Params to tune are declared in params_holder.h
	int iLastX = -1;
    int iLastY = -1;
    
	//Convert the captured frame from BGR to HSV
	cv::Mat imgHSV, imgOriginal = normals.clone();
	cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); 

	//Threshold the image
	cv::Mat imgThresholded;
	cv::inRange(imgHSV, cv::Scalar(_iLowH, _iLowS, _iLowV), cv::Scalar(_iHighH, _iHighS, _iHighV), imgThresholded);  
	
	//morphological operation
	//opening (removes small objects from the foreground)
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(_seg_erosion_size, _seg_erosion_size)) );
	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(_seg_erosion_size, _seg_erosion_size)) ); 
	//closing (removes small holes from the foreground)
	cv::dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(_seg_erosion_size, _seg_erosion_size)) ); 
	cv::erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(_seg_erosion_size, _seg_erosion_size)) );

	// Find c-space
	//cv::erode( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(100, 100)) ); 
    
	//show the thresholded image
	// cv::imshow("segmented Ground", imgThresholded);
    // cv::waitKey(1); 
	return imgThresholded;
}
