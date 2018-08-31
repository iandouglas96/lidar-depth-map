#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define INITIAL_RESOLUTION_SCALE_H 4
#define INITIAL_RESOLUTION_SCALE_V 16

using namespace cv;

//Camera calibration stuff
bool haveCalib = false;
bool haveImage = false;
cv::Matx33f projection = {0,0,0,0,0,0,0,0,1};
Eigen::Affine3f sensorPose;
cv::Mat last_image;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //ROS_INFO("Got image...");

    cv_bridge::CvImagePtr image_msg = cv_bridge::toCvCopy(*msg);

    //cv::imshow("img", image_msg->image);
    cv::cvtColor(image_msg->image, last_image, cv::COLOR_GRAY2BGR);
    //cv::waitKey(1);

    haveImage = true;
}

void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (haveCalib) {
        //ROS_INFO("Got Pointcloud...");

        pcl::PointCloud<pcl::PointXYZ>::Ptr pc =
            pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pc);

        //ROS_INFO_STREAM(*pc << "\n");

        pcl::RangeImagePlanar rangeImage;
        
        rangeImage.createFromPointCloudWithFixedSize(*pc, 1224/INITIAL_RESOLUTION_SCALE_H, 1024/INITIAL_RESOLUTION_SCALE_V,
                                        projection(0,2)/INITIAL_RESOLUTION_SCALE_H, projection(1,2)/INITIAL_RESOLUTION_SCALE_V, 
                                        projection(0,0)/INITIAL_RESOLUTION_SCALE_H, projection(1,1)/INITIAL_RESOLUTION_SCALE_V, 
                                        sensorPose, pcl::RangeImagePlanar::LASER_FRAME);

        cv::Mat image = cv::Mat(rangeImage.height, rangeImage.width, CV_32F, 0.0);

        for (int y=0; y<rangeImage.height; y+=1) {
            for (int x=0; x<rangeImage.width; x+=1) {
                if (rangeImage.getPoint(y*rangeImage.width + x).range > 0) {
                    image.at<float>(y,x) = (rangeImage.getPoint(y*rangeImage.width + x).range);
                    //ROS_INFO("depth: %f",image.at<float>(y,x));
                }
            }
        }

        //std::cout << rangeImage << "\n";
        cv::Mat image_overlay, image_color, image_grey, image_scaled, inpaint_mask;
        cv::resize(image,image_scaled,cv::Size(1224, 1024),0,0,INTER_AREA);//resize image

        //Create mask to find holes
        inpaint_mask = cv::Mat::zeros(image_scaled.size(), CV_8UC1);
        inpaint_mask.setTo(255, image_scaled == 0);
        //Don't treat the top and bottom of the image as "holes"
        cv::floodFill(inpaint_mask, cv::Point(1,1), 0);
        cv::floodFill(inpaint_mask, cv::Point(1,image_scaled.size().height-1), 0);

        cv::imshow("mask", inpaint_mask);

        //Inpaint holes
        clock_t t;
        t = clock();
        cv::inpaint(image_scaled, inpaint_mask, image_scaled, 1, cv::INPAINT_NS);
        t = clock() - t;
        printf("Inpainting took %f sec\n", ((float)t)/CLOCKS_PER_SEC);

        //Format for grayscale display
        image_scaled.convertTo(image_grey,CV_8UC1, 255 / 6, 0); 
        cv::applyColorMap(image_grey, image_color, cv::COLORMAP_JET);

        if (haveImage) {
            cv::addWeighted(image_color, 0.5, last_image, 0.5, 0.0, image_overlay);
            cv::imshow("depth", image_overlay);
        } else {
            cv::imshow("depth", image_color);
        }

        cv::waitKey(1);
    }  
}

//Load projection matrix
void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (!haveCalib) {
        projection(0,0) = msg->P[0];
        projection(0,2) = msg->P[2];
        projection(1,1) = msg->P[5];
        projection(1,2) = msg->P[6];
        haveCalib = true;
    }

    //ROS_INFO_STREAM(projection << "\n");
}

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "lidar_depth_map");
    ros::NodeHandle n("lidar_depth_map");

    ROS_INFO("Starting up...\n");

    //Set up lidar and camera calibration
    //sensorPose.translation() << 0,0,0;
    //sensorPose.translation() << 0.341161,  -0.13896, -0.0614301; //Translation matrix
    sensorPose.translation() << 0.0796279, -0.011135, 0; //+back, +right, -up, reorder to 1,2,0
    sensorPose.linear().setIdentity();
    sensorPose.linear() <<  0.9957,   -0.0573,    0.0726,
                            0.0566,    0.9983,    0.0117,
                           -0.0731,   -0.0076,    0.9973; //Rotation matrix
    //sensorPose.linear() = sensorPose.linear().inverse(); 
    
    //Setup listeners and callbacks
    ros::Subscriber image_sub = n.subscribe("image", 100, imageCallback);
    ros::Subscriber pc_sub = n.subscribe("pc", 100, pcCallback);
    ros::Subscriber cam_info_sub = n.subscribe("cam_info", 100, camInfoCallback);

    ros::spin();
}