#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "pcd_publisher");

    // Create a ROS node handle
    ros::NodeHandle nh("~");

    // Read the PCD file path from the parameter
    std::string pcdFilePath;
    if (!nh.getParam("pcd_file_path", pcdFilePath))
    {
        ROS_ERROR("Failed to retrieve PCD file path parameter");
        return 1;
    }

    // Create a publisher for the modified point cloud
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_pcd", 10);

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilePath, *cloud);

    // Reduce the height of each point by 1.2 meters
    for (auto& point : cloud->points)
    {
        point.z -= 1.2;
    }

    // Convert the point cloud to ROS message
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "world";  // Set the desired frame ID
    // Set the publish rate to 1 Hz
    ros::Rate rate(1.0);

    while (ros::ok())
    {
        // Publish the modified point cloud
        pub.publish(msg);

        // Spin once and sleep to maintain the publish rate
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
