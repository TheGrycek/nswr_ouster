#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

void callback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg)
{
    // Convert message to point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloudMsg, *cloud);

    float SCANNER_COLS = 2048;
    float SCANNER_ROWS = 128;
    float MAX_V_ANGLE = 22.5;
    float MIN_V_ANGLE = -22.5;

    // Create range image
    cv::Mat range_img = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_32FC1);
    range_img = 0;

    // Create intensity image
    cv::Mat intensity_img = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_32FC1);
    intensity_img = 0;

    for (int idx = 0; idx < cloud->points.size(); idx++)
    {
        // Get point from cloud
        pcl::PointXYZI point = cloud->points[idx];

        // Remove invalid points
        if (!pcl_isfinite(point.x) ||
            !pcl_isfinite(point.y) ||
            !pcl_isfinite(point.z))
            continue;

        // Get intensity value
        float intensity = point.intensity;

        // Create intensity image






        // Create range image



        
       
    }

    // Normalize range_img to 0 - 255

 
    // Write images to file
    

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nswr_ouster");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("os1_cloud_node/points", 1, callback);
    ros::spin();
    return 0;
}