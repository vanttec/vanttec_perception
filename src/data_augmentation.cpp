//------------------------------------------------------------------------------
// @file: data_augmentation.cpp
// @created on: March 18th, 2019
// @author: Ivana Collado
//
// @brief:Contains the implementations for the Data Augmentation class.
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "data_augmentation.h"

// CLASS FUNCTION IMPLEMENTATION  ----------------------------------------------
DataAugmentation::DataAugmentation(){
  // Empty Body
}

DataAugmentation::~DataAugmentation(){
  // Empty body
}

// FUNCTIONS -----------------------------------------------------------------
void DataAugmentation::Read(const std::string &path){
  //Read image from path
  in_ = cv::imread( path, 1 );
  if (! in_.data ) 
    {
        std::cout << "Could not open or find the image.\n";
        //return -1; // unsuccessful
    }
  cv::namedWindow("Original",CV_WINDOW_NORMAL); 
  cv::imshow("Original",in_);
}

void DataAugmentation::AverageFilter(const int &kernel){
  out_ = in_.clone();
  cv::blur( in_, out_, cv::Size( kernel, kernel ), cv::Point(-1,-1) );
  cv::namedWindow("Average Filter",CV_WINDOW_NORMAL); 
  cv::imshow("Average Filter",out_);
  cv::imwrite("../../imgs/average.png",out_);
}

void DataAugmentation::SaltPepper(const float percentage){
  cv::Mat pepperized_ = in_.clone();
  int total = percentage * pepperized_.cols * pepperized_.rows;
  for(int i = 0; i<total; i++){
    //Choose a random pixel between 0 - (im.cols-1)
    //and 0 - (im.rows-1)
    int a =rand()%pepperized_.cols;
    int b =rand()%pepperized_.rows;
    //Randomly change pixel color to black or white
    int color = rand()%2 ? 255:0;
    //Apply color
    if(pepperized_.channels() == 1){
        pepperized_.at<uchar>(b,a) = color;
    }
    else if(pepperized_.channels() == 3){
        pepperized_.at<cv::Vec3b>(b,a)[0] = color;
        pepperized_.at<cv::Vec3b>(b,a)[1] = color;
        pepperized_.at<cv::Vec3b>(b,a)[2] = color;
    }
  }
  cv::namedWindow("Salt_and_Pepper",CV_WINDOW_KEEPRATIO);
  cv::imshow("Salt_and_Pepper",pepperized_);
  cv::imwrite("../../imgs/salt_pepper.png",pepperized_);
}