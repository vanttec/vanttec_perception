//------------------------------------------------------------------------------
// @file: data_augmentation.cpp
// @created on: March 18th, 2019
// @author: Ivana Collado
// @co-author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
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

void DataAugmentation::GaussianBlur(const int &kernel){
  out_ = in_.clone();
  cv::GaussianBlur( in_, out_, cv::Size( kernel, kernel ), 0, 0 );
  cv::namedWindow("Gaussian Blur Filter",CV_WINDOW_NORMAL); 
  cv::imshow("Gaussian Blur Filter",out_);
  cv::imwrite("../../imgs/gaussian.png",out_);
}

void DataAugmentation::Brightness(const int brightness){
  // Full explanation can be found in: 
  // https://docs.opencv.org/3.4/d3/dc1/tutorial_basic_linear_transform.html
  int i,j;
  out_ = cv::Mat::zeros( in_.size(), in_.type() );
  double alpha_ = 1.0;
  int beta_ = brightness;
  for( int j = 0; j < in_.rows; j++ ) {
      for( int i = 0; i < in_.cols; i++ ) {
          for( int c = 0; c < in_.channels(); c++ ) {
              out_.at<cv::Vec3b>(j,i)[c] =
                cv::saturate_cast<uchar>( alpha_*in_.at<cv::Vec3b>(j,i)[c] + beta_ );
          }
      }
  }
  cv::namedWindow("Brightness_value: "+std::to_string(beta_),CV_WINDOW_NORMAL); 
  cv::imshow("Brightness_value: "+std::to_string(beta_), out_);
  cv::imwrite("../../imgs/Brightness:"+ std::to_string(beta_)+".png", out_);
}