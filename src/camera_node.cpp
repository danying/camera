/*
 * camera_node.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: danying
 */

#include "camera/camera.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "usbcam");
    ros::NodeHandle nh, pnh("~");

    USBCamera cam(nh,pnh);
    ros::spin();
    return 0;
}
