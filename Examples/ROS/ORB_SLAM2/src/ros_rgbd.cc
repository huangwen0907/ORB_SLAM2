/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
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

#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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

    // get the current pose from the RGBD image
    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    if (pose.empty()) {
        return;
    }

    /* Global left handed coordinate system*/
    static cv::Mat pose_prev = cv::Mat::eye(4,4,CV_32F);
    static cv::Mat word_lh = cv::Mat::eye(4,4,CV_32F);

    /*Matrix to flip signs of sinus in rotation matrix*/
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4)<<1,-1,-1,1,
                                                            -1, 1,-1,1,
                                                            -1,-1,1,1,
                                                            1,1,1,1 );

    // pose = prev_pose * T
    cv::Mat translation = (pose* pose_prev.inv()).mul(flipSign);
    word_lh = word_lh * translation;

    pose_prev = pose.clone();

    /*Transform into global right handed coordinate system, publish in ROS*/
    /*
    *   SE(3) = |  sR    t|
    *           |   0    1|
    *
    * */
    tf::Matrix3x3 cameraRotation_rh(-word_lh.at<float>(0,0),word_lh.at<float>(0,1),word_lh.at<float>(0,2),
                                    -word_lh.at<float>(1,0),word_lh.at<float>(1,1),word_lh.at<float>(1,2),
                                    word_lh.at<float>(2,0),-word_lh.at<float>(2,1),-word_lh.at<float>(2,2));

    tf::Vector3 cameraTranlation_rh(word_lh.at<float>(0,3),word_lh.at<float>(1,3),-word_lh.at<float>(2,3));

    // Rotate 270deg about x and y to get ENU, x--> Forward; y --> left; z-->up
    const tf::Matrix3x3 rotation270degXZ(0,1,0,
                                        0,0,1,
                                        1,0,0);

    static tf::TransformBroadcaster br;

    tf::Matrix3x3 globalRotation = cameraRotation_rh * rotation270degXZ;
    tf::Vector3 globalTranlation = cameraTranlation_rh * rotation270degXZ;
    tf::Transform transform = tf::Transform(globalRotation,globalTranlation);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"camera_link", "camera_pose"));

}


