#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>

/************************************************* 
    Function:       main
    Description:    function entrance
    Input:          None 
    Output:         None 
    Return:         return 0
    *************************************************/
int main(int argc, char **argv)
{
    int iscamera;
    std::string video_path;
    cv::FileStorage setting_fs(ros::package::getPath("detector") + "/capture_node/param/param.xml", cv::FileStorage::READ);
    setting_fs["camera"] >> iscamera;
    setting_fs["video_path"] >> video_path;
    video_path = ros::package::getPath("detector") + video_path;

	ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/video", 1);

	cv::VideoCapture capture;

	if(!iscamera) 
        capture.open(video_path);
	else 
        capture.open(0);

	if(capture.isOpened())
		std::cout<<"INFO: Video or Camera load sucessfully"<<std::endl;
	else
		std::cout<<"ERROR: Cannot find Video file or Open camera fail"<<std::endl;

	cv::Mat frame, output;

	sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(5);

	while (capture.read(frame))
	{
		capture>>frame; 
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
	}
}
