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

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h> 
#include <tf/tf.h> 
#include <tf/transform_datatypes.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include <opencv2/core/eigen.hpp>

using namespace std;

class ImageGrabber
{
public:
    ros::NodeHandle nh;
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    nav_msgs::Path camerapath;
    cv_bridge::CvImageConstPtr cv_ptr;
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("Pose", 100);
    ros::Publisher pub_odom= nh.advertise<nav_msgs::Odometry> ("Odometry", 10); 
    ros::Publisher pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 10); 
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/infra1/image_rect_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/infra2/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw_SE3f;

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw_SE3f = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        Tcw_SE3f = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    cv::Mat Tcw;
    Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
    cv::eigen2cv(Tcw_Matrix, Tcw);

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
    
    // -----------------------------------
    // Publish tf transform
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = cv_ptrLeft->header.stamp;
    transformStamped.header.frame_id = "camera_infra2_optical_frame"; 
    transformStamped.child_frame_id = "world";
    transformStamped.transform.translation.x = tcw.at<float>(0);
    transformStamped.transform.translation.y = tcw.at<float>(1);
    transformStamped.transform.translation.z = tcw.at<float>(2);
    // -----------------------------------
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
    transformStamped.transform.rotation.x = q[0];
    transformStamped.transform.rotation.y = q[1];
    transformStamped.transform.rotation.z = q[2];
    transformStamped.transform.rotation.w = q[3];
    // -----------------------------------

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
    header.stamp = cv_ptrLeft->header.stamp;
    header.seq = cv_ptrLeft->header.seq;
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

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);

}


