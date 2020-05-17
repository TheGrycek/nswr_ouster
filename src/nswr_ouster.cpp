#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <cmath>
#include <math.h>

float biggest_dist = 0.0;
float smallest_dist = 50.0;
double biggest_vertical = 0.0;
double smallest_vertical = 180.0;
float biggest_intensity = 0.0;
float smallest_intensity = 100000.0;
float intensity;
float intensity_scaled;
float distance;
float distance_scaled;

cv::Mat invert_image(cv::Mat const& input)
{
    return 255 - input;
}

void callback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg)
{
    std::cout<<"Jesteśmy w callbacku" << std::endl;
    // Convert message to point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloudMsg, *cloud);  //  obiekt cloud

    float SCANNER_COLS = 2048;
    float SCANNER_ROWS = 128;
    float MAX_V_ANGLE = 22.5;
    float MIN_V_ANGLE = -22.5;

    // Create range image
    cv::Mat range_img = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_8UC1);
    range_img = 0;

    cv::Mat range_img_scaled = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_8UC1);
    range_img_scaled = 0;
//    std::cout<<"Oto rozmiary obrazu do zapisu"<<std::endl;
//    std::cout << range_img<<std::endl;

    // Create intensity image
    cv::Mat intensity_img = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_8UC1);
    intensity_img = 0;

    cv::Mat intensity_img_scaled = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_8UC1);
    intensity_img_scaled = 0;

    for (int idx = 0; idx < cloud->points.size(); idx++)
    {
        // Get point from cloud
        pcl::PointXYZI point = cloud->points[idx];

        // Remove invalid points
        if (!pcl_isfinite(point.x) ||
            !pcl_isfinite(point.y) ||
            !pcl_isfinite(point.z))
            continue;

        float x = point.x;
        float y = point.y;
        float z = point.z;

        float vertical_angle = atan2(z, sqrt(pow(x, 2) + pow(y, 2))) * (180.00/M_PI);

        if(vertical_angle > biggest_vertical)
        {
            biggest_vertical = vertical_angle;
        }

        if(vertical_angle < smallest_vertical)
        {
            smallest_vertical = vertical_angle;
        }

        float distance = float(round(sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0))*10)/10);

        if(distance > biggest_dist)
        {
            biggest_dist = distance;
        }

        if(distance < smallest_dist)
        {
            smallest_dist = distance;
        }

        float intensity = point.intensity;
        intensity = float(round((((intensity - 0)/(3652 - 0))*(255 - 0) + 0)*10)/10);

        if(intensity > biggest_intensity)
        {
            biggest_intensity = intensity;
        }

        if(intensity < smallest_intensity)
        {
            smallest_intensity = intensity;
        }
    }

    std::cout <<"Biggest vertical: "<< biggest_vertical<<std::endl;
    std::cout <<"Smallest vertical: "<< smallest_vertical<<std::endl;
    std::cout <<"Biggest dist: "<< biggest_dist<<std::endl;
    std::cout <<"Smallest dist: "<< smallest_dist<<std::endl;
    std::cout <<"Smallest intensity: "<< smallest_intensity<<std::endl;
    std::cout <<"Biggest intesity: "<< biggest_intensity<<std::endl;


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
        intensity = point.intensity;
        intensity_scaled = point.intensity;

        if(intensity_scaled > 255.0)
        {
            intensity_scaled = 255.0;
        }

        double x = point.x;
        double y = point.y;
        double z = point.z;
        int row;
        int col;

        // Create intensity image

        double vertical_angle = abs((atan2(z, sqrt(pow(x, 2) + pow(y, 2))) * (180.0/M_PI)) - biggest_vertical);
        float horizontal_angle = (atan2(y, x) * (180.00/M_PI)) + 180;
        distance = float(round(sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0))*10)/10);
        distance_scaled = distance;

        if(distance_scaled > 50)
        {
            distance_scaled = 50;
        }

        double factor = (SCANNER_ROWS)/45;
        row = ((vertical_angle) * factor);
        col = round((horizontal_angle) * (SCANNER_COLS/360));
        intensity_img.at<uchar>(row, col) = int(intensity);
        range_img.at<uchar>(row, col) = round(((distance - 0)/(biggest_dist - 0)) * (255 - 0) + 0);

        intensity_img_scaled.at<uchar>(row, col) = int(intensity_scaled);
        range_img_scaled.at<uchar>(row, col) = round((((distance_scaled - 0)/(50 - 0)) * (255 - 0) + 0));

    }

//    range_img = invert_image(range_img);
//    cv::imshow("Intensity Image",intensity_img);
//    cv::imshow("Range Image", range_img);
//    cv::imshow("Intensity Image Scaled",intensity_img_scaled);
//    cv::imshow("Range Image Scaled", range_img_scaled);

    cv::imwrite("intensity_image.png", intensity_img_scaled);
    cv::imwrite("range_image.png", range_img_scaled);
//    cv::waitKey();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nswr_ouster");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("os1_cloud_node/points", 1, callback);
    ros::spin();
    return 0;
}
