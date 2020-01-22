#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <algorithm>
#include <iostream>
#include <vector>


class CavityDetector
{
    public:
    int print_position,show_image,publish_position,show_box;
    
    CavityDetector();

    void preProcess(cv::Mat &input);
    cv::RotatedRect detectCavity(cv::Mat input,std::vector< std::vector<cv::Point2i> > &target_contour, cv::RotatedRect &target_rect);
    
    int darkThreshold(cv::Mat src,int dark_threshold, int offset);
    int thresholdOtsu(cv::Mat &image);
    void drawDetection(cv::Mat &src, cv::RotatedRect target_rect,std::vector< std::vector<cv::Point2i> > target_contour);
    
    private:

};
