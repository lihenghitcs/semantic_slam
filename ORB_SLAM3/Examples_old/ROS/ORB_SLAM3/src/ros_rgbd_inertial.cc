/**
* This file is part of ORB-SLAM3
*
* Copydepth (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copydepth (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

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

float shift = 0;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ros::NodeHandle nh;
    nav_msgs::Path camerapath;
    cv_bridge::CvImageConstPtr cv_ptr;

    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("Pose", 100);
    ros::Publisher pub_odom= nh.advertise<nav_msgs::Odometry> ("Odometry", 10); 
    ros::Publisher pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 10); 

    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageRgb(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageDepth(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgRgbBuf, imgDepthBuf;
    std::mutex mBufMutexRgb,mBufMutexDepth;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "RGBD_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 2 )
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD_inertial path_to_vocabulary path_to_settings " << endl;
    ros::shutdown();
    return 1;
  }

  std::string sbRect = "false";
  if(argc==5)
  {
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,sbRect == "true",bEqual);
  
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
  }

  shift = fsSettings["IMU.shift"];

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/camera/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img_rgb = n.subscribe("/camera/color/image_raw", 100, &ImageGrabber::GrabImageRgb,&igb);
  ros::Subscriber sub_img_depth = n.subscribe("/camera/aligned_depth_to_color/image_raw", 100, &ImageGrabber::GrabImageDepth,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();

  return 0;
}



void ImageGrabber::GrabImageRgb(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRgb.lock();
  if (!imgRgbBuf.empty())
    imgRgbBuf.pop();
  imgRgbBuf.push(img_msg);
  mBufMutexRgb.unlock();
}

void ImageGrabber::GrabImageDepth(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexDepth.lock();
  if (!imgDepthBuf.empty())
    imgDepthBuf.pop();
  imgDepthBuf.push(img_msg);
  mBufMutexDepth.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
    return cv_ptr->image.clone();
  

}

void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while(1)
  {
    cv::Mat imRgb, imDepth;
    double tImRgb = 0, tImDepth = 0;
    if (!imgRgbBuf.empty()&&!imgDepthBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tImRgb = imgRgbBuf.front()->header.stamp.toSec();
      tImDepth = imgDepthBuf.front()->header.stamp.toSec();

      this->mBufMutexDepth.lock();
      while((tImRgb-tImDepth)>maxTimeDiff && imgDepthBuf.size()>1)
      {
        imgDepthBuf.pop();
        tImDepth = imgDepthBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexDepth.unlock();

      this->mBufMutexRgb.lock();
      while((tImDepth-tImRgb)>maxTimeDiff && imgRgbBuf.size()>1)
      {
        imgRgbBuf.pop();
        tImRgb = imgRgbBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRgb.unlock();

      if((tImRgb-tImDepth)>maxTimeDiff || (tImDepth-tImRgb)>maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if(tImRgb>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      this->mBufMutexRgb.lock();
      imRgb = GetImage(imgRgbBuf.front());
      imgRgbBuf.pop();
      this->mBufMutexRgb.unlock();

      this->mBufMutexDepth.lock();
      imDepth = GetImage(imgDepthBuf.front());
      imgDepthBuf.pop();
      this->mBufMutexDepth.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImRgb+shift)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
      {
        mClahe->apply(imRgb,imRgb);
        mClahe->apply(imDepth,imDepth);
      }

      // if(do_rectify)
      // {
      //   cv::remap(imRgb,imRgb,M1l,M2l,cv::INTER_LINEAR);
      //   cv::remap(imDepth,imDepth,M1r,M2r,cv::INTER_LINEAR);
      // }

      cv::Size dsize = cv::Size(640, 480);
      cv::Mat rgb_resize;
      cv::resize(imRgb,rgb_resize,dsize,0,0, cv::INTER_NEAREST);
      cv::Mat depth_resize;
      cv::resize(imDepth,depth_resize,dsize,0,0, cv::INTER_NEAREST);
      
      Sophus::SE3f Tcw_SE3f = mpSLAM->TrackRGBD(rgb_resize,depth_resize,tImRgb,vImuMeas);
      cv::Mat Tcw;
      Eigen::Matrix4f Tcw_Matrix = Tcw_SE3f.matrix();
      cv::eigen2cv(Tcw_Matrix, Tcw);

      if(Tcw.empty())
        continue;
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
      transformStamped.header.stamp = cv_ptr->header.stamp;
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
      header.stamp =cv_ptr->header.stamp;
      header.seq = cv_ptr->header.seq;
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
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}