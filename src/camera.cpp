/*
 * camera.cpp
 *
 *  Created on: Aug 8, 2013
 *      Author: danying
 */

#include "camera/camera.h"


USBCamera::USBCamera(ros::NodeHandle & nh, ros::NodeHandle & paramnh)
{
    _nh = nh;
    _ph = paramnh;
    _ph.param<int>("width",width,int(640));
    _ph.param<int>("height",height,int(480));
    _ph.param<int>("fps",fps,int(30));
    _ph.param<int>("device_id",device_id,int(0));
    _ph.param<std::string>("frame_id",frame_id,std::string("/camera"));
    _ph.param<std::string>("camera_name",camera_name,std::string("/quickcam"));
    _ph.param<std::string>("cam_info_url",cam_info_url,std::string("file:///home/danying/ROSWorkspace/camera/camera_info/2014/logitech.yaml"));

    camera_info_mgr = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(_nh,camera_name,cam_info_url));

    it = new image_transport::ImageTransport(_nh);
    image_pub = it->advertiseCamera("/image_raw",1);

    isStarted = false;
    cap.open(device_id);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,height);

    if(cap.isOpened())
    {
        isStarted = true;
        image_thread = boost::thread(boost::bind(&USBCamera::publishImage, this));
    }
}

USBCamera::~USBCamera()
{
    isStarted = false;
    image_thread.interrupt();
    image_thread.join();
    cap.release();
    cv::destroyAllWindows();
    delete it;
    camera_info_mgr.reset();
}

void USBCamera::setCameraParameters()
{
    cam_info = camera_info_mgr->getCameraInfo();
}

void USBCamera::resetCameraParameters(){}

void USBCamera::publishImage()
{
    USBCamera::setCameraParameters();

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv::Mat frame;

    while(isStarted)
    {
        cap >> frame;
        ros::Time capture_time = ros::Time::now();
        cv_ptr->image = frame;

        try
        {
            image = *(cv_ptr->toImageMsg());
            image.encoding = sensor_msgs::image_encodings::BGR8;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        image.header.frame_id = cam_info.header.frame_id = frame_id;
        image.header.stamp = cam_info.header.stamp = capture_time;
        image.height = cam_info.height = frame.rows;
        image.width = cam_info.width = frame.cols;

        image_pub.publish(image,cam_info);
    }
}



