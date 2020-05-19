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

void DataAugmentation::Scaling_ROI(const float ratio){
  if(ratio < 1){
    cv::Mat temp = in_.clone();
    cv::Size size;
    cv::Point offset;
    // Width and  height of ROI
    int width = std::round(ratio*in_.cols);
    int height = std::round(ratio*in_.rows);
    for(int i=0; i<=2; i++){
      // Get offset of the 3 ROI's based on the original image
      offset.x = std::round(in_.cols*(1-ratio)/2);
      offset.y = std::round(in_.rows*(1-ratio)/2*i);
      size.height = height;
      size.width = width;
      //Crop the ROI from original image
      cv::Rect ROI(offset, size);
      out_ = temp(ROI);  
      //Resize ROI back to the original image
      cv::resize(out_, out_, cv::Size(in_.rows, in_.cols), 
                                                    0, 0, CV_INTER_LINEAR);
      cv::namedWindow("Cropped image",CV_WINDOW_NORMAL); 
      cv::imshow("Cropped image",out_);
      switch (i){
      case 0:
        cv::imwrite("../../imgs/scale_upper_x_"+std::to_string(ratio)+".png",
                                                                        out_);
        break;
      case 1:
      cv::imwrite("../../imgs/scale_middle_x_"+std::to_string(ratio)+".png",
                                                                       out_);
        break;
      case 2:
      cv::imwrite("../../imgs/scale_lower_x_"+std::to_string(ratio)+".png",
                                                                      out_);
        break;
      default:
        break;
      }
    }
  }
void DataAugmentation::ContrastBrightness(const double contrast, const int brightness){
  // Full explanation can be found in: 
  // https://docs.opencv.org/3.4/d3/dc1/tutorial_basic_linear_transform.html
  int i,j,c;
  out_ = cv::Mat::zeros( in_.size(), in_.type() );
  for(j = 0; j < in_.rows; j++ ) {
      for(i = 0; i < in_.cols; i++ ) {
          for(c = 0; c < in_.channels(); c++ ) {
              out_.at<cv::Vec3b>(j,i)[c] =
              cv::saturate_cast<uchar>( contrast*in_.at<cv::Vec3b>(j,i)[c] + brightness);
          }
      }
  }
  cv::namedWindow("Brightness_value: "+std::to_string(brightness),CV_WINDOW_NORMAL); 
  cv::imshow("Brightness_value: "+std::to_string(brightness), out_);
  cv::imwrite("../../imgs/brightness:"+ std::to_string(brightness)+".png", out_);
}