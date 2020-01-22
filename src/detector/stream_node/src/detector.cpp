#include "detector.hpp"

CavityDetector::CavityDetector()
{
    cv::FileStorage setting_fs(ros::package::getPath("detector") + "/stream_node/param/param.xml", cv::FileStorage::READ);
    setting_fs["print_position"] >> print_position;
    setting_fs["publish_position"] >> publish_position;
    setting_fs["show_image"] >> show_image;
    setting_fs["show_box"] >> show_box;
}

void CavityDetector::preProcess(cv::Mat &input)
{
    cv::Mat src,out;
    input.copyTo(src);
    bilateralFilter(src, out,25, 25 * 2, 25 / 2,cv::BORDER_DEFAULT);    // Bilateral Filters

    // split channels
    /*std::vector<cv::Mat> channels;
    cv::split(out,channels);
    cv::Mat Red = channels.at(2);
    cv::Mat Blue = channels.at(0);
    for (int i = 0; i < Red.rows; i++){
        for (int j = 0; j < Red.cols; j++){
            if (Red.at<uchar>(i, j) > 60 && (Red.at<uchar>(i, j) > 0.4 * Blue.at<uchar>(i, j))){ //color threshold
                Red.at<uchar>(i, j) = 0;
            }
            else
                Red.at<uchar>(i, j) = 255;
        }
    }
    cv::morphologyEx(Red, Red, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5))); //dilate to get more obvious contour
    */

    cv::Mat binery;
    cv::cvtColor(out,out,CV_BGR2GRAY);
    int thresh = darkThreshold(out,50,30);      // select thresh
    //std::cout<<thresh<<std::endl;
    cv::threshold(out,binery,thresh,255,cv::THRESH_BINARY);     // binery
    binery = 255 - binery;      // reverse image
    //imshow("binery",binery);
    binery.copyTo(input);
}

cv::RotatedRect CavityDetector::detectCavity(cv::Mat input,std::vector< std::vector<cv::Point2i> > &target_contour, cv::RotatedRect &target_rect)
{
    cv::Mat src;
    input.copyTo(src);
    preProcess(src);

    std::vector<cv::Vec4i> hierarchy;
    std::vector< std::vector<cv::Point2i> > contours;

    findContours(src, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    double rectsize=0;
    int max_index=0;

    for (int i = 0; i < contours.size(); i++){
        cv::RotatedRect bound_rect=minAreaRect(contours[i]);
        double bound_size = bound_rect.size.height*bound_rect.size.width;
        if(bound_size >= rectsize)
        {
            target_rect = bound_rect;
            rectsize = bound_size;
            max_index = i;
        }
    }
    target_contour.push_back(contours[max_index]);
    if(print_position) std::cout<<target_rect.center<<std::endl;
    return target_rect;
}

void CavityDetector::drawDetection(cv::Mat &src, cv::RotatedRect target_rect,std::vector< std::vector<cv::Point2i> > target_contour)
{
    cv::Mat mask;
    src.copyTo(mask);
    cv::drawContours(mask,target_contour,-1,cv::Scalar(150,255,30),-1);
    cv::addWeighted(mask,0.5,src,0.7,5,src);
    cv::Point2f vertex[4];
	target_rect.points(vertex);
    if(show_box==1)
        for (int i = 0; i < 4; i++)
			line(src, vertex[i], vertex[(i + 1) % 4], cv::Scalar(0, 0, 255), 2, 8);
    cv::circle(src,target_rect.center,3,cv::Scalar(20,0,200),-1);
}


int CavityDetector::thresholdOtsu(cv::Mat &image)
 {
	int thresh;
	int pixNumber = image.cols * image.rows;        // pixels of the image
	int pixCount[256] = { 0 };                      // pixel for each gray

	// counts number of each gray  
	for (int i = 0; i < image.rows; i++)
	{
		for (int j = 0; j < image.cols; j++)
		{
			pixCount[image.at<uchar>(i,j)]++;            
		}
	}
	
	// Get highest varance
	float PixBackground = 0;
	float GrayBackground = 0;
	float GrayAverageBackground = 0;
	float PixForeground = 0;
	float GrayForeground = 0;
	float GrayAverageForeground = 0;
	float InterclassVariance = 0;
	float InterclassVarianceMax = 0;
	// [1] outer loop for chosing threshld and caculte varance between classes
	for (int i = 0; i < 256; i++)                  
	{
		PixBackground = PixForeground = GrayBackground = GrayForeground = GrayAverageBackground = GrayAverageForeground = InterclassVariance = 0;
		//[2] inner loop for distiguish back and front ground
		for (int j = 0; j < 256; j++)
		{
			if (j <= i)         //background
			{
				PixBackground += pixCount[j];
				GrayBackground += j * pixCount[j];
			}
			else                //frontground
			{
				PixForeground += pixCount[j];
				GrayForeground += j * pixCount[j];
			}
		}
		GrayAverageBackground = GrayBackground / PixBackground;
		GrayAverageForeground = GrayForeground / PixForeground;

        //InterclassVariance
		InterclassVariance = (float)(PixBackground *PixForeground* pow((GrayAverageBackground - GrayAverageForeground), 2));    
		
        if (InterclassVariance > InterclassVarianceMax)
		{
			InterclassVarianceMax = InterclassVariance;
			thresh = i;
		}
	}
	return thresh;
}
 
int CavityDetector::darkThreshold(cv::Mat src, int dark_threshold, int offset)
{
    int darkest = thresholdOtsu(src);
    std::vector<int> dark_count(darkest+1);
    for (int i = 0; i < src.rows; i++){
        for (int j = 0; j < src.cols; j++){
            int scalar=src.at<uchar>(i, j);
            if(scalar<darkest){
                dark_count[scalar]++;
            }
        }
    }
    for(int n=dark_count.size();n>0;n--){
        if(dark_count[n]>dark_threshold && n<=darkest){
            darkest=n;
        }
    }
    return darkest+offset;
}
