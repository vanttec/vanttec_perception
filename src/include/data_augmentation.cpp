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

void DataAugmentation::Hue(const u_char min_hue, const u_char max_hue, u_char step){
  out_ = in_.clone();
  cv::Mat HSV;
  cv::cvtColor(in_, HSV, CV_BGR2HSV);
  int i,j;
  // Apply Hue changes
  for(int hue = min_hue; hue <= max_hue; hue += step){
    for(j=0; j < HSV.rows; j++){
      for(i=0; i < HSV.cols; i++){
        // cv::Vec3b --> Vec<uchar,3> Uchar type vector of 3 elements
        // uchar --> typedef unsigned char (an unsigned 1 byte integer)
        HSV.at<cv::Vec3b>(j,i)[0] = hue;
      }
    }
    cv::cvtColor(HSV, out_, CV_HSV2BGR);
    cv::imshow("BGR_hue: "+std::to_string(hue), out_);
    cv::imwrite("../../imgs/BGR_hue:"+ std::to_string(hue)+".png", out_);
  }
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

void DataAugmentation::CombiningFilters(){
  std::string path;
  std::cout << "Enter folder path:";
  std::cin >> path;
}
