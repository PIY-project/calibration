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

namespace calibration {

class AverageCalibratorTestPcd
{
    private:
        // node handles
        ros::NodeHandle nh_;
        ros::NodeHandle priv_nh_;

        // services
        ros::ServiceServer srv_calibrate_;
        ros::ServiceServer srv_calibrate_pcd_;
        ros::ServiceServer srv_add_pose_;
        ros::ServiceServer srv_remove_pose_;
        ros::ServiceServer srv_clear_;

        ros::Publisher pub_cloud_;
        ros::Publisher pub_cloud_icp_;
        ros::Subscriber sub_point_cloud_;

        // it is very useful to have a listener and broadcaster to know where all frames are
        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster tf_broadcaster_;

        // the looked transform
        tf::Transform transform_;

        std::string frame_id_;
        std::string child_frame_id_;
        std::string ar_marker_frame_;
        std::string calibrator_frame_;
        
        std::string calibration_name_;

        sensor_msgs::PointCloud2 point_cloud_;
        rpwc_utils_pcl rpwc_pcl_;

        double samples_; //how many samples to keep
  
        //vectors to store published translation and rotations (in axis angle, to avoid averaging quaternions)
        std::vector<double> kept_translation_x_first_;
        std::vector<double> kept_translation_y_first_;
        std::vector<double> kept_translation_z_first_;
        std::vector<Eigen::Quaterniond> kept_quaternion_first_;
        std::vector<double> kept_translation_x_second_;
        std::vector<double> kept_translation_y_second_;
        std::vector<double> kept_translation_z_second_;
        std::vector<Eigen::Quaterniond> kept_quaternion_second_;
        //translation and rotation to be published (as the average)
        std::vector<double> published_translation_;
        std::vector<double> published_rotation_;
        std::vector<tf::Transform> transform_vector_;
    public:
        bool calibrate(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        bool calibrate_pcd(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        bool addPose(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        bool clearvec(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
        bool removePose(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

        void callback_point_cloud (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

        void publishTf();
        bool recordTf(int i);

        // constructor
        AverageCalibratorTestPcd(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
        {

            // load the parameters
            nh_.param<std::vector<double> >("translation", published_translation_, {0, 0, 0});
            nh_.param<std::vector<double> >("rotation", published_rotation_, {0, 0, 0, 1});
            nh_.param<std::string>("frame_id", frame_id_, "/camera_depth_optical_frame");
            nh_.param<std::string>("ar_marker_frame", ar_marker_frame_, "/ar_marker_60");
            nh_.param<std::string>("child_frame_id", child_frame_id_, "/phase_space_world");
            nh_.param<std::string>("calibrator_frame", calibrator_frame_, "/calibrator");
            nh_.param<std::string>("calibration_name", calibration_name_, "asus_phase_space");
            
            samples_ = 50;
            kept_translation_x_first_.resize(samples_, 0);
            kept_translation_y_first_.resize(samples_, 0);
            kept_translation_z_first_.resize(samples_, 0);
            kept_quaternion_first_.resize(samples_);
            kept_translation_x_second_.resize(samples_, 0);
            kept_translation_y_second_.resize(samples_, 0);
            kept_translation_z_second_.resize(samples_, 0);
            kept_quaternion_second_.resize(samples_);

            // init the calibration to the identity to publish something
            transform_.setOrigin( tf::Vector3( published_translation_.at(0), published_translation_.at(1), published_translation_.at(2) ) );
            transform_.setRotation( tf::Quaternion( published_rotation_.at(0), published_rotation_.at(1), published_rotation_.at(2), published_rotation_.at(3) ) );

            // advertise service
            srv_add_pose_ = nh_.advertiseService(nh_.resolveName("add"), &AverageCalibratorTestPcd::addPose, this);
            srv_remove_pose_ = nh_.advertiseService(nh_.resolveName("remove"), &AverageCalibratorTestPcd::removePose, this);
            srv_calibrate_ = nh_.advertiseService(nh_.resolveName("calibrate"), &AverageCalibratorTestPcd::calibrate, this);
            srv_calibrate_pcd_ = nh_.advertiseService(nh_.resolveName("calibrate_pcd"), &AverageCalibratorTestPcd::calibrate_pcd, this);
            srv_clear_ = nh_.advertiseService(nh_.resolveName("clear"), &AverageCalibratorTestPcd::clearvec, this);

            pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/robot_pcd", 1);
            pub_cloud_icp_ = nh.advertise<sensor_msgs::PointCloud2>("/icp_result", 1);
            sub_point_cloud_ = nh.subscribe("/camera/depth_registered/points", 1, &AverageCalibratorTestPcd::callback_point_cloud, this);

        }

        //! Empty stub
        ~AverageCalibratorTestPcd() {}

};
bool AverageCalibratorTestPcd::recordTf(int i)
{
    // 1. get the first transformation
    tf::StampedTransform first_transformation;
    try
    {
      tf_listener_.lookupTransform(frame_id_, ar_marker_frame_,ros::Time(0), first_transformation);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }

    // 2. get the second transformation
    tf::StampedTransform second_transformation;
    try
    {
      tf_listener_.lookupTransform(child_frame_id_, calibrator_frame_,ros::Time(0), second_transformation);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    /*
    //store transforms
    //erase first element
    kept_translation_x_first_.erase(kept_translation_x_first_.begin());
    kept_translation_y_first_.erase(kept_translation_y_first_.begin());
    kept_translation_z_first_.erase(kept_translation_z_first_.begin());
    kept_quaternion_first_.erase(kept_quaternion_first_.begin());     
    kept_translation_x_second_.erase(kept_translation_x_second_.begin());
    kept_translation_y_second_.erase(kept_translation_y_second_.begin());
    kept_translation_z_second_.erase(kept_translation_z_second_.begin());
    kept_quaternion_second_.erase(kept_quaternion_second_.begin());     
    // std::cout <<"kept_translation_x_first_ "<<kept_translation_x_first_[0]<<" "<<kept_translation_x_first_[samples_-1]<<std::endl;
*/
    //populate by pushing back size is now again == samples_
    kept_translation_x_first_.at(i)=(first_transformation.getOrigin()[0]);
    kept_translation_y_first_.at(i)=(first_transformation.getOrigin()[1]);
    kept_translation_z_first_.at(i)=(first_transformation.getOrigin()[2]);
    kept_quaternion_first_.at(i)=(Eigen::Quaterniond(first_transformation.getRotation().getW(), first_transformation.getRotation().getX(), 
          first_transformation.getRotation().getY(), first_transformation.getRotation().getZ()));

    kept_translation_x_second_.at(i)=(second_transformation.getOrigin()[0]);
    kept_translation_y_second_.at(i)=(second_transformation.getOrigin()[1]);
    kept_translation_z_second_.at(i)=(second_transformation.getOrigin()[2]);

    kept_quaternion_second_.at(i)=(Eigen::Quaterniond(second_transformation.getRotation().getW(), second_transformation.getRotation().getX(), 
          second_transformation.getRotation().getY(), second_transformation.getRotation().getZ()));
    return true;
    
}

bool AverageCalibratorTestPcd::calibrate( std_srvs::Empty::Request &request, std_srvs::Empty::Response &response )
{
    double n = transform_vector_.size();
    if(n<1) {

    	std::cout << " no poses!!!" << std::endl;
    	return false;
    }

    double tx(0),ty(0),tz(0);
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d A_tmp = Eigen::Matrix4d::Zero();

    Eigen::Quaterniond q;
    for(int i = 0; i<n; i++)
    {
    q = Eigen::Quaterniond(transform_vector_.at(i).getRotation().getW(), transform_vector_.at(i).getRotation().getX(), 
          transform_vector_.at(i).getRotation().getY(), transform_vector_.at(i).getRotation().getZ());

     tx    +=   transform_vector_.at(i).getOrigin()[0];
     ty    +=   transform_vector_.at(i).getOrigin()[1];
     tz    +=   transform_vector_.at(i).getOrigin()[2];
     

      A_tmp <<  q.w()*q.w(), 		q.w()*q.x(), 	q.w()*q.y(), 	q.w()*q.z(),
       	   		q.w()*q.x(), 	q.x()*q.x(), 		q.x()*q.y(), 	q.x()*q.z(),
               	q.w()*q.y(), 	q.x()*q.y(),  	q.y()*q.y(), 		q.y()*q.z(),
           		q.w()*q.z(), 	q.x()*q.z(), 	q.y()*q.z(),  	q.z()*q.z();
	  A = A+  A_tmp;
    }
    tx /= n;
    ty /= n;
    tz /= n;

    A = 1/n*A;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(A);

    Eigen::VectorXcd v=es.eigenvectors().col(es.eigenvectors().cols()-1);
    Eigen::Quaterniond q_avg(v[0].real(),v[1].real(),v[2].real(),v[3].real());
	

    //tf::Transform first_tran;

    transform_.setOrigin( tf::Vector3( tx,ty,tz) );
	q_avg.normalize();
    transform_.setRotation( tf::Quaternion(q_avg.x(), q_avg.y(), q_avg.z(), q_avg.w()));


    Eigen::Matrix3d temp_R(q_avg);


    std::string path = ros::package::getPath("calibration");
    std::string file = path + "/config/" + calibration_name_ + ".yaml";
    //std::cout << file.c_str() << std::endl;
    std::ofstream f;
    f.open(file.c_str());
    if (f.is_open())
    {
      f << "# This file contains the values obtained by the calibration package" << std::endl;
      f << "# Results are written in the form that they can be directly sent to a static_transform_publisher node" << std::endl;
      f << "# Recall its usage use static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds)" << std::endl;
      f << "translation: [" << transform_.getOrigin()[0] << ", " << 
                          transform_.getOrigin()[1] << ", " << 
                          transform_.getOrigin()[2]<<"]" << std::endl;
      f << "rotation: [" << transform_.getRotation().getX() << ", " << 
                            transform_.getRotation().getY() << ", " << 
                            transform_.getRotation().getZ() << ", " << 
                            transform_.getRotation().getW() << "]" << std::endl;
      f << "T: [" << std::to_string(temp_R(0,0)) << ", " << std::to_string(temp_R(0,1)) << ", " << std::to_string(temp_R(0,2)) << ", " << std::to_string(tx) << ", "
                  << std::to_string(temp_R(1,0)) << ", " << std::to_string(temp_R(1,1)) << ", " << std::to_string(temp_R(1,2)) << ", " << std::to_string(ty) << ", "
                  << std::to_string(temp_R(2,0)) << ", " << std::to_string(temp_R(2,1)) << ", " << std::to_string(temp_R(2,2)) << ", " << std::to_string(tz) << ", "
                  << std::to_string(0.0) << ", " << std::to_string(0.0) << ", " << std::to_string(0.0) << ", " << std::to_string(1.0) << "]" << std::endl;

      f << "frame_id: " << frame_id_.c_str() << std::endl;
      f << "child_frame_id: " << child_frame_id_.c_str() << std::endl;
      f.close();
    }
    return true;
}

bool AverageCalibratorTestPcd::calibrate_pcd( std_srvs::Empty::Request &request, std_srvs::Empty::Response &response )
{
  tf::StampedTransform transformation_cam2robotLink;
  try
  {
    tf_listener_.lookupTransform(frame_id_, "/setup1/L5",ros::Time(0), transformation_cam2robotLink);

    Eigen::Matrix4d T_cam2robotLink(Eigen::Matrix4d::Identity());
    T_cam2robotLink.block<3,1>(0,3) << transformation_cam2robotLink.getOrigin()[0], transformation_cam2robotLink.getOrigin()[1], transformation_cam2robotLink.getOrigin()[2];
    T_cam2robotLink.block<3,3>(0,0) = Eigen::Matrix3d(Eigen::Quaterniond(transformation_cam2robotLink.getRotation().getW(), transformation_cam2robotLink.getRotation().getX(), transformation_cam2robotLink.getRotation().getY(), transformation_cam2robotLink.getRotation().getZ()));
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/nuk1/star_ws/src/abb_wrapper/gofa_description/meshes/L5.pcd", *target_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file .pcd \n");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_result(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*target_cloud, *target_cloud_result, T_cam2robotLink);
    // declare the output variable instances
    pcl::PCLPointCloud2 outputPCL;
    sensor_msgs::PointCloud2 output;

    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *target_cloud_result ,outputPCL);

    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);
    output.header.frame_id = frame_id_;
    pub_cloud_.publish(output);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
    //rpwc_pcl_.cloud2_to_pcl(point_cloud_ , pclCloudPtr);
    rpwc_pcl_.pcl_prefilter(point_cloud_, pclCloudPtr, 0.01, -0.80, 0.80, -0.80, 0.80, -0.80, 0.80);
    std::string path = ros::package::getPath("calibration") + "/pcd_test/test_pcd.pcd";
    pcl::io::savePCDFileASCII (path, *pclCloudPtr);

    // Estimate normals for object
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_result_normal (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pclCloudPtr_normal (new pcl::PointCloud<pcl::PointNormal>);

    pcl::copyPointCloud(*target_cloud_result,*target_cloud_result_normal);
    pcl::copyPointCloud(*pclCloudPtr,*pclCloudPtr_normal);


		pcl::console::print_highlight ("Estimating object normals...\n");
		pcl::NormalEstimationOMP<pcl::PointNormal,pcl::PointNormal> nest_obj;
		nest_obj.setRadiusSearch (0.01);
		//nest_obj.setRadiusSearch (0.005);
		nest_obj.setInputCloud (target_cloud_result_normal);
		nest_obj.compute (*target_cloud_result_normal);

		// Estimate normals for scene
		pcl::console::print_highlight ("Estimating scene normals...\n");
		pcl::NormalEstimationOMP<pcl::PointNormal,pcl::PointNormal> nest;
		// nest.setRadiusSearch (0.01);
		nest.setRadiusSearch (0.005);
		nest.setInputCloud (pclCloudPtr_normal);
		nest.compute (*pclCloudPtr_normal);

    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    icp.setMaximumIterations (5000);
    icp.setInputSource (pclCloudPtr_normal);
    icp.setInputTarget (target_cloud_result_normal);
    pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned_final (new pcl::PointCloud<pcl::PointNormal>);
    icp.align (*object_aligned_final);
    if (icp.hasConverged ())
    {
      std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
      Eigen::Matrix4f transformation_2 = icp.getFinalTransformation ();
      pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_2 (0,0), transformation_2 (0,1), transformation_2 (0,2));
      pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation_2 (1,0), transformation_2 (1,1), transformation_2 (1,2));
      pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_2 (2,0), transformation_2 (2,1), transformation_2 (2,2));
      pcl::console::print_info ("\n");
      pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_2 (0,3), transformation_2 (1,3), transformation_2 (2,3));
      pcl::console::print_info ("\n");

      pcl::PCLPointCloud2 outputPCL_icp;
      sensor_msgs::PointCloud2 output_icp;

      // convert to pcl::PCLPointCloud2
      pcl::toPCLPointCloud2( *object_aligned_final ,outputPCL_icp);

      // Convert to ROS data type
      pcl_conversions::fromPCL(outputPCL_icp, output_icp);
      output_icp.header.frame_id = frame_id_;
      pub_cloud_icp_.publish(output_icp);
    }
    else
    {
      PCL_ERROR ("\nICP has not converged.\n");
      return (-1);
    }


  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }

  return true;
}

bool AverageCalibratorTestPcd::addPose( std_srvs::Empty::Request &request, std_srvs::Empty::Response &response )
{
	ros::Rate rate(10.0); //go at 100Hz
    int num_t = 0;
    while(ros::ok() && num_t<samples_){	
    	if(recordTf(num_t)) num_t++;
		
		rate.sleep();
		ros::spinOnce();
    }
    
    tf::Transform first_tran, second_tran, calib_tran;
    // 1. get the frist transformation as the average of those in memory
    double tx(0),ty(0),tz(0);
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d A_tmp = Eigen::Matrix4d::Zero();

    Eigen::Quaterniond q;
    for (int i =0; i<samples_; ++i)
    {
      tx += kept_translation_x_first_[i];
      ty += kept_translation_y_first_[i];
      tz += kept_translation_z_first_[i];
      q = kept_quaternion_first_.at(i);
      A_tmp <<  q.w()*q.w(), 		q.w()*q.x(), 	q.w()*q.y(), 	q.w()*q.z(),
       	   		q.w()*q.x(), 	q.x()*q.x(), 		q.x()*q.y(), 	q.x()*q.z(),
               	q.w()*q.y(), 	q.x()*q.y(),  	q.y()*q.y(), 		q.y()*q.z(),
           		q.w()*q.z(), 	q.x()*q.z(), 	q.y()*q.z(),  	q.z()*q.z();

	  A = A+  A_tmp;
    }
    tx /= samples_;
    ty /= samples_;
    tz /= samples_;

    A = 1/samples_*A;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(A);
    int maxElementIndex;
    int maxRow;
    ///Check if correct!!! I have to select the eigenvector for the maximum eigenvalue
    Eigen::VectorXcd v = es.eigenvectors().col(es.eigenvectors().cols()-1);
    Eigen::Quaterniond q_first(v[0].real(),v[1].real(),v[2].real(),v[3].real());

    first_tran.setOrigin( tf::Vector3( tx,ty,tz) );
    q_first.normalize();

    first_tran.setRotation( tf::Quaternion(q_first.x(), q_first.y(), q_first.z(), q_first.w()));

    // 2. get the second transformation as the average f those in memory
    tx=ty=tz=0;
    A = Eigen::Matrix4d::Zero();

    for (int i =0; i<samples_; ++i)
    {
      tx += kept_translation_x_second_[i];
      ty += kept_translation_y_second_[i];
      tz += kept_translation_z_second_[i];
      q = kept_quaternion_second_.at(i);

      A_tmp <<  q.w()*q.w(), 		q.w()*q.x(), 	q.w()*q.y(), 	q.w()*q.z(),
       	   		q.w()*q.x(), 	q.x()*q.x(), 		q.x()*q.y(), 	q.x()*q.z(),
               	q.w()*q.y(), 	q.x()*q.y(),  	q.y()*q.y(), 		q.y()*q.z(),
           		q.w()*q.z(), 	q.x()*q.z(), 	q.y()*q.z(),  	q.z()*q.z();

	  A = A+  A_tmp;

    }
    tx /= samples_;
    ty /= samples_;
    tz /= samples_;

    A = 1/samples_*A;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es1(A);
    es1.eigenvalues().maxCoeff(&maxRow,&maxElementIndex);
    v = es1.eigenvectors().col(es1.eigenvectors().cols()-1);;

    Eigen::Quaterniond q_second(v[0].real(),v[1].real(),v[2].real(),v[3].real());
            
    second_tran.setOrigin( tf::Vector3( tx,ty,tz) );
    q_second.normalize();

    second_tran.setRotation( tf::Quaternion(q_second.x(), q_second.y(), q_second.z(), q_second.w()));

    // 3. since the reference frame is the same, we just need to inverse and multiply
    
    calib_tran = first_tran*second_tran.inverse();
    transform_vector_.push_back(calib_tran);
    // // 4. write the transform on a yaml file
    // std::string path = ros::package::getPath("calibration");
    // std::string file = path + "/config/" + calibration_name_ + ".yaml";
    // //std::cout << file.c_str() << std::endl;
    // std::ofstream f;
    // f.open(file.c_str());
    // if (f.is_open())
    // {
    //   f << "# This file contains the values obtained by the calibration package" << std::endl;
    //   f << "# Results are written in the form that they can be directly sent to a static_transform_publisher node" << std::endl;
    //   f << "# Recall its usage use static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds)" << std::endl;
    //   f << "translation: [" << transform_.getOrigin()[0] << ", " << 
    //                       transform_.getOrigin()[1] << ", " << 
    //                       transform_.getOrigin()[2]<<"]" << std::endl;
    //   f << "rotation: [" << transform_.getRotation().getX() << ", " << 
    //                         transform_.getRotation().getY() << ", " << 
    //                         transform_.getRotation().getZ() << ", " << 
    //                         transform_.getRotation().getW() << "]" << std::endl;
    //   f << "frame_id: " << frame_id_.c_str() << std::endl;
    //   f << "child_frame_id: " << child_frame_id_.c_str() << std::endl;
    //   f.close();
    // }
    return true;
}

bool AverageCalibratorTestPcd::removePose(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
	transform_vector_.pop_back();
	return true;
}

bool AverageCalibratorTestPcd::clearvec(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
	transform_vector_.clear();
return true;
}

void AverageCalibratorTestPcd::callback_point_cloud (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	point_cloud_ = *cloud_msg;
}
// this function is called as fast as ROS can from the main loop directly
void AverageCalibratorTestPcd::publishTf()
{
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), frame_id_, child_frame_id_)); //from asus to phase space
}

} // namespace calibration

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_calibrator_node");
    ros::NodeHandle nh;

    calibration::AverageCalibratorTestPcd node(nh);
    ros::Rate rate(10.0); //go at 100Hz
  
    //wait a bit to let tf start publishing
    //boost::this_thread::sleep (boost::posix_time::microseconds (3000000));
    usleep(3000000);
    while(nh.ok())
    {
      //node.recordTf();
      node.publishTf();
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
