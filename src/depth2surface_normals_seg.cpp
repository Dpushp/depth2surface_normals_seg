#include<depth_segmentation.h>

/* DyParams */
int d_erosion_size, seg_erosion_size, d_gaussian_blur_size;
int iLowH = 30, iLowS = 150, iLowV = 60;
int iHighH = 110, iHighS = 255, iHighV = 255;

void callback(depth2surface_normals_seg::DenoiseConfig &config, uint32_t level) {
    d_erosion_size = config.d_erosion_size;
    d_gaussian_blur_size = config.d_gaussian_blur_size;
    seg_erosion_size = config.seg_erosion_size;
    iLowH = config.iLowH;
    iLowS = config.iLowS; 
    iLowV = config.iLowV;
    iHighH = config.iHighH;
    iHighS = config.iHighS;
    iHighV = config.iHighV;
    ROS_INFO("Reconfigure Request: %d %d %d %d %d %d %d %d %d", 
            d_erosion_size, d_gaussian_blur_size, seg_erosion_size, 
            iHighH, iHighS, iHighV, iLowH, iLowS, iLowV);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "depth2surface_normals_seg");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<depth2surface_normals_seg::DenoiseConfig> server;
  dynamic_reconfigure::Server<depth2surface_normals_seg::DenoiseConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  DepthSegmentation depth_segmentation(nh);

  while(ros::ok()) {
        depth_segmentation.updateConfig(d_erosion_size, d_gaussian_blur_size, 
        seg_erosion_size, iLowH, iLowS, iLowV, iHighH, iHighS, iHighV);
        ros::spinOnce();
  }
  return 0;
}