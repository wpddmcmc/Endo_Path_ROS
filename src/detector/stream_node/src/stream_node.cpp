#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "detector.hpp"
#include "detector/Detector_Info.h"

class StreamNode{
public:
    StreamNode()
    {
        cv::namedWindow("view", 0);
        cv::namedWindow("output", 0);
        cv::startWindowThread();
        image_transport::ImageTransport it(nh);
        sub = it.subscribe("camera/video", 1, &StreamNode::imageCallback,this);
        detector::Detector_Info ros_info_;
        detect_pub_ = nh.advertise<detector::Detector_Info>("cavity_data", 1000);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv::Mat frame, output;
        CavityDetector detector;
        std::vector<std::vector<cv::Point2i>> target_contour;
        cv::RotatedRect target_rect;

        cv::namedWindow("view", false);
        cv::resizeWindow("view", cv::Size(640, 480));
        cv::namedWindow("output", false);
        cv::resizeWindow("output", cv::Size(640, 480));
        try
        {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::resize(frame, frame, cv::Size(640, 480));

            if (detector.show_image)
                cv::imshow("view", frame);

            frame.copyTo(output);
            detector.detectCavity(output, target_contour, target_rect);
            detector.drawDetection(output, target_rect, target_contour);

            if (detector.show_image)
                cv::imshow("output", output);

            char key = cv::waitKey(10);
            if (key == 27)
                exit(0);
            if(detector.publish_position=1)
            {
                ros_info_.center_x = target_rect.center.x;
                ros_info_.center_y = target_rect.center.y;
                detect_pub_.publish(ros_info_);
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
private:
     ros::NodeHandle nh;
     image_transport::Subscriber sub;
     detector::Detector_Info ros_info_;
     ros::Publisher detect_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    StreamNode stream_node;
    
    ros::spin();
    cv::destroyWindow("view");
}   