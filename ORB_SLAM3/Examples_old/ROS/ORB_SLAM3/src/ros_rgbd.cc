/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

// For TF
#include"../../../include/Converter.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/PoseStamped.h> 
#include <tf/tf.h> 
#include <tf/transform_datatypes.h> 
#include"../../../include/Converter.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/eigen.hpp>

using namespace std;

class ImageGrabber
{
public:
    ros::NodeHandle nh;
    nav_msgs::Path camerapath;

    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("Pose", 100);
	ros::Publisher pub_odom= nh.advertise<nav_msgs::Odometry> ("Odometry", 10); 
	ros::Publisher pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 10); 

    //ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM),nh("~"){
        lastStamp = ros::Time::now();
        frameCounter = 0;
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;

private:
    ros::Time lastStamp;
    int frameCounter;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw_SE3f = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    cv::Mat Tcw;
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix, Tcw);

    frameCounter++;
    // Print frame rate every x second
    if((cv_ptrRGB->header.stamp - lastStamp).toSec() >= 5.0)
    {
        float fps = frameCounter / (cv_ptrRGB->header.stamp - lastStamp).toSec();
        lastStamp = cv_ptrRGB->header.stamp;
        ROS_INFO("Frames per second: %f", fps);
        frameCounter = 0;
    }

    if(Tcw.empty())
        return;
    cv::Mat Twc =Tcw.inv();
    cv::Mat tWC=  Twc.rowRange(0,3).col(3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);


    cv::Mat RWC= Twc.rowRange(0,3).colRange(0,3); 
    //cv::Mat TWC=orbslam->mpTracker->mCurrentFrame.mTcw.inv();  
    //cv::Mat RWC= Tcw.rowRange(0,3).colRange(0,3).t();//Tcw.rowRange(0,3).colRange(0,3);  
    //cv::Mat tWC=  -RWC*Tcw.rowRange(0,3).col(3);//Tcw.rowRange(0,3).col(3);

    Eigen::Matrix<double,3,3> eigMat ;
    eigMat <<RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
                    RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
                    RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2);
    Eigen::Quaterniond q2(eigMat);

    // Publish tf transform
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = cv_ptrRGB->header.stamp;
    transformStamped.header.frame_id = "camera_color_optical_frame"; 
    transformStamped.child_frame_id = "world";
    transformStamped.transform.translation.x = tcw.at<float>(0);
    transformStamped.transform.translation.y = tcw.at<float>(1);
    transformStamped.transform.translation.z = tcw.at<float>(2);
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    transformStamped.transform.rotation.x = q[0];
    transformStamped.transform.rotation.y = q[1];
    transformStamped.transform.rotation.z = q[2];
    transformStamped.transform.rotation.w = q[3];

    br.sendTransform(transformStamped);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id ="world";

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
    vector<float> q1 = ORB_SLAM3::Converter::toQuaternion(Rwc);

    tf::Transform new_transform;
    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

    tf::Quaternion quaternion(q1[0], q1[1], q1[2], q1[3]);
    new_transform.setRotation(quaternion);

    tf::poseTFToMsg(new_transform, pose.pose);
    pub_pose.publish(pose);

    std_msgs::Header header ;
    header.stamp =msgRGB->header.stamp;
    header.seq = msgRGB->header.seq;
    header.frame_id="world";

    nav_msgs::Odometry odom_msg;
    odom_msg.pose.pose.position.x=twc.at<float>(0);
    odom_msg.pose.pose.position.y=twc.at<float>(1);			 
    odom_msg.pose.pose.position.z=twc.at<float>(2);

    odom_msg.pose.pose.orientation.x=q2.x();
    odom_msg.pose.pose.orientation.y=q2.y();
    odom_msg.pose.pose.orientation.z=q2.z();
    odom_msg.pose.pose.orientation.w=q2.w();

    odom_msg.header=header;
    odom_msg.child_frame_id="base_link";


    geometry_msgs::PoseStamped tcw_msg; 					
    tcw_msg.pose.position.x=tWC.at<float>(0);
    tcw_msg.pose.position.y=tWC.at<float>(1);			 
    tcw_msg.pose.position.z=tWC.at<float>(2);
    tcw_msg.pose.orientation.x=q2.x();
    tcw_msg.pose.orientation.y=q2.y();
    tcw_msg.pose.orientation.z=q2.z();
    tcw_msg.pose.orientation.w=q2.w();
    tcw_msg.header=header;


    camerapath.header =header;
    camerapath.poses.push_back(tcw_msg);     
    pub_odom.publish(odom_msg);
    pub_camerapath.publish(camerapath);
}


