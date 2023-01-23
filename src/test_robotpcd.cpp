// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/thread/thread.hpp>
#include <Eigen/Eigenvalues>
#include <rpwc_utils_pcl.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_robotpcd_node");
    ros::NodeHandle nh;

    ros::Rate rate(10.0);

    ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/robot_pcd", 1);

    sensor_msgs::PointCloud2 output;
    tf::TransformListener tf_listener;
    Eigen::Matrix4d T_base2L5(Eigen::Matrix4d::Identity());

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/nuk1/star_ws/src/abb_wrapper/gofa_description/meshes/L5.pcd", *target_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file .pcd \n");
        return false;
    }


    while(nh.ok())
    {
        tf::StampedTransform transformation_base2L5;
        try
        {
            tf_listener.lookupTransform("/setup1/robot1/base", "/setup1/robot1/L5",ros::Time(0), transformation_base2L5);
            T_base2L5.block<3,1>(0,3) << transformation_base2L5.getOrigin()[0], transformation_base2L5.getOrigin()[1], transformation_base2L5.getOrigin()[2];
            T_base2L5.block<3,3>(0,0) = Eigen::Matrix3d(Eigen::Quaterniond(transformation_base2L5.getRotation().getW(), transformation_base2L5.getRotation().getX(), transformation_base2L5.getRotation().getY(), transformation_base2L5.getRotation().getZ()));
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_result(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*target_cloud, *target_cloud_result, T_base2L5);
            // declare the output variable instances
            pcl::PCLPointCloud2 outputPCL;

            // convert to pcl::PCLPointCloud2
            pcl::toPCLPointCloud2( *target_cloud_result ,outputPCL);

            // Convert to ROS data type
            pcl_conversions::fromPCL(outputPCL, output);
            output.header.frame_id = "/setup1/robot1/base";
            pub_cloud.publish(output);
            // transformation_base2L5.
            // T_base2L5
            
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            //return false;
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
