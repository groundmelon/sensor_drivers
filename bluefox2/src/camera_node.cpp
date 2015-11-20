#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <queue>

#include <unistd.h>
#include <errno.h>
#include <mavros_msgs/CamIMUStamp.h>

#include "camera.h"

ros::Publisher pub_img;

int img_cnt = 0, trigger_cnt = 0;

BluefoxManager bluefoxManager;

queue<mavros_msgs::CamIMUStampConstPtr> trigger_buf;

void trigger_callback(const mavros_msgs::CamIMUStampConstPtr &trigger_msg)
{
    trigger_buf.push(trigger_msg);
    printf("size of trigger: %lu\n", trigger_buf.size());
}

void process()
{
    vector<vector<char>> data;
    if (!trigger_buf.empty() && bluefoxManager.ready())
    {
        sensor_msgs::ImagePtr img_msg(new sensor_msgs::Image);
        img_msg->height = 480 * bluefoxManager.getImgCnt();
        img_msg->width = 752;
        img_msg->step = 752;
        img_msg->encoding = sensor_msgs::image_encodings::MONO8;

        img_msg->header.stamp = ros::Time(trigger_buf.front()->frame_stamp);
        trigger_buf.pop();
        img_msg->data = std::move(bluefoxManager.getImg());
        pub_img.publish(img_msg);
        printf("after publish, size of queue: %lu\n", trigger_buf.size());
    }
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv)
//-----------------------------------------------------------------------------
{
    ros::init(argc, argv, "bluefox2");
    ros::NodeHandle n("~");

    pub_img = n.advertise<sensor_msgs::Image>("image", 1000);

    ros::Subscriber sub_trigger = n.subscribe("trigger", 1000, trigger_callback);

    ros::Rate r(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
