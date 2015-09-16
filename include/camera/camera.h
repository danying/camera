/*
 * camera.h
 *
 *  Created on: Aug 8, 2013
 *      Author: danying
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>


class USBCamera{

private:

    ros::NodeHandle _nh,_ph;

    image_transport::ImageTransport *it;
    image_transport::CameraPublisher image_pub;

    sensor_msgs::Image image;
    sensor_msgs::CameraInfo cam_info;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_mgr;

    cv::VideoCapture cap;
    bool isStarted;

    int width;
    int height;
    int fps;
    int device_id;
    std::string frame_id;
    std::string cam_info_url;
    std::string camera_name;

    boost::thread image_thread;

public:

    USBCamera(ros::NodeHandle & nh, ros::NodeHandle & paramnh);
    ~USBCamera();

    void setCameraParameters();
    void resetCameraParameters();
    void publishImage();
};



#endif /* CAMERA_H_ */
