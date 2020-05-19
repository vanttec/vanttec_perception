//------------------------------------------------------------------------------
// @file: data_augmentation.cpp
// @created on: March 18th, 2020
// @modified: May 18th, 2020
// @author: Ivana Collado
// @mail: ivanacollado@gmail.com
// @co-author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
// @brief: Contains the implementations for the Data Augmentation class.
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

void DataAugmentation::GaussianBlur(const int &kernel){
  out_ = in_.clone();
  cv::GaussianBlur( in_, out_, cv::Size( kernel, kernel ), 0, 0 );
  cv::namedWindow("Gaussian Blur Filter",CV_WINDOW_NORMAL); 
  cv::imshow("Gaussian Blur Filter",out_);
  cv::imwrite("../../imgs/gaussian.png",out_);
}

void DataAugmentation::SaltPepper(const float percentage){
  out_ = in_.clone();
  int total = percentage * out_.cols * out_.rows;
  int column_pixel, row_pixel, color;
  for(int i = 0; i<total; i++){
    //Choose a random pixel between 0 and (out_.cols-1)
    //and 0 and (out_.rows-1)
    column_pixel = rand()%out_.cols;
    row_pixel = rand()%out_.rows;
    //Randomly change pixel color to black or white
    color = rand()%2 ? 255:0;
    //Apply color
    if(out_.channels() == 1){
        out_.at<uchar>(row_pixel,column_pixel) = color;
    }
    else if(out_.channels() == 3){
        out_.at<cv::Vec3b>(row_pixel,column_pixel)[0] = color;
        out_.at<cv::Vec3b>(row_pixel,column_pixel)[1] = color;
        out_.at<cv::Vec3b>(row_pixel,column_pixel)[2] = color;
    }
  }
  cv::namedWindow("Salt_and_Pepper",CV_WINDOW_KEEPRATIO);
  cv::imshow("Salt_and_Pepper",out_);
  cv::imwrite("../../imgs/salt_pepper.png",out_);
}