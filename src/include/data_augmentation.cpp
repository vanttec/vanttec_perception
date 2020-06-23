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
#include <time.h>
#include <typeinfo>

// CLASS FUNCTION IMPLEMENTATION  ----------------------------------------------
DataAugmentation::DataAugmentation(){
  // Random seed
  srand(time(NULL));
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
  // cv::namedWindow("Original",CV_WINDOW_NORMAL); 
  // cv::imshow("Original",in_);
}

void DataAugmentation::GaussianBlur(const int &kernel){
  out_.push_back(in_.clone());
  cv::GaussianBlur( in_, out_[0], cv::Size( kernel, kernel ), 0, 0 );
  // cv::namedWindow("Gaussian Blur Filter",CV_WINDOW_NORMAL); 
  // cv::imshow("Gaussian Blur Filter",out_);
  // cv::imwrite("../../Filtered_imgs/gaussian.jpg",out_);
}

void DataAugmentation::Hue(const u_char min_hue, const u_char max_hue, u_char step){
  cv::Mat HSV, BGR;
  cv::cvtColor(in_, HSV, CV_BGR2HSV);
  int i,j,k=0;
  // Apply Hue changes
  for(int hue = min_hue; hue <= max_hue; hue += step){
    for(j=0; j < HSV.rows; j++){
      for(i=0; i < HSV.cols; i++){
        // cv::Vec3b --> Vec<uchar,3> Uchar type vector of 3 elements
        // uchar --> typedef unsigned char (an unsigned 1 byte integer)
        HSV.at<cv::Vec3b>(j,i)[0] = hue;
      }
    }
    out_.push_back(BGR);
    cv::cvtColor(HSV, out_[k++], CV_HSV2BGR);
    // cv::imshow("BGR_hue: "+std::to_string(hue), out_[k++]);
    // cv::imwrite("../../Filtered_imgs/BGR_hue:"+ std::to_string(hue)+".jpg", out_);
  }
}

void DataAugmentation::SaltPepper(const float percentage){
  out_.push_back(in_.clone());
  int total = percentage * out_[0].cols * out_[0].rows;
  int column_pixel, row_pixel, color;
  for(int i = 0; i<total; i++){
    //Choose a random pixel between 0 and (out_.cols-1)
    //and 0 and (out_.rows-1)
    column_pixel = rand()%out_[0].cols;
    row_pixel = rand()%out_[0].rows;
    //Randomly change pixel color to black or white
    color = rand()%2 ? 255:0;
    //Apply color
    if(out_[0].channels() == 1){
        out_[0].at<uchar>(row_pixel,column_pixel) = color;
    }
    else if(out_[0].channels() == 3){
        out_[0].at<cv::Vec3b>(row_pixel,column_pixel)[0] = color;
        out_[0].at<cv::Vec3b>(row_pixel,column_pixel)[1] = color;
        out_[0].at<cv::Vec3b>(row_pixel,column_pixel)[2] = color;
    }
  }
  // cv::namedWindow("Salt_and_Pepper",CV_WINDOW_KEEPRATIO);
  // cv::imshow("Salt_and_Pepper",out_[0]);
  // cv::imwrite("../../Filtered_imgs/salt_pepper.jpg",out_[0]);
}

void DataAugmentation::Scaling_ROI(const float ratio){
  if(ratio < 1){
    cv::Mat temp = in_.clone(), img;
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
      img = temp(ROI);  
      //Resize ROI back to the original image
      cv::resize(img, img, cv::Size(in_.rows, in_.cols), 
                                                    0, 0, CV_INTER_LINEAR);
      // cv::namedWindow("Cropped image",CV_WINDOW_NORMAL); 
      // cv::imshow("Cropped image",out_);
      switch (i){
      case 0:
      // cv::imwrite("../../Filtered_imgs/scale_upper_x_"+std::to_string(ratio)+".jpg",
      //                                                                   out_);
        out_.push_back(img);
        break;
      case 1:
      // cv::imwrite("../../Filtered_imgs/scale_middle_x_"+std::to_string(ratio)+".jpg",
      //                                                                  out_);
        out_.push_back(img);
        break;
      case 2:
      // cv::imwrite("../../Filtered_imgs/scale_lower_x_"+std::to_string(ratio)+".jpg",
      //                                                                 out_);
        out_.push_back(img);
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
  out_.push_back( cv::Mat::zeros( in_.size(), in_.type() ) );
  for(j = 0; j < in_.rows; j++ ) {
      for(i = 0; i < in_.cols; i++ ) {
          for(c = 0; c < in_.channels(); c++ ) {
              out_[0].at<cv::Vec3b>(j,i)[c] =
              cv::saturate_cast<uchar>( contrast*in_.at<cv::Vec3b>(j,i)[c] + brightness);
          }
      }
  }
  // cv::namedWindow("Brightness_value: "+std::to_string(brightness),CV_WINDOW_NORMAL); 
  // cv::imshow("Brightness_value: "+std::to_string(brightness), out_);
  // cv::imwrite("../../Filtered_imgs/brightness:"+ std::to_string(brightness)+".jpg", out_);
}

void DataAugmentation::read_directory(const std::string path, std::vector<std::string>& images){
  DIR* dirp = opendir(path.c_str());
  struct dirent * dp;
  std::string file;
  std::size_t found;
  int num_files = 0, i = 1;
  while ((dp = readdir(dirp)) != NULL) {
    file = dp->d_name;
    found = file.find(".png");
    if(found != std::string::npos){
      images.push_back(dp->d_name);
      std::cout<<std::to_string(i++) + " image: "<<dp->d_name<<std::endl;
      // num_files++;
    }
  }
  closedir(dirp);
}

void DataAugmentation::CombiningFilters(std::vector<std::string> images){
  DataAugmentation data;
  int combination = 7 ; //rand()%7 + 1;
  int kernel_size, brightness, j;
  float noise_percentage, scaling_ratio;
  double contrast; 
  u_char min_hue, max_hue, step;

  std::cout<<std::to_string(images.size())<<std::endl;
  for(int i=0; i<images.size(); i++){
    std::cout<<images[i]<<"\n"; 
    data.Read("../../imgs/" + images[i]);
    // Define parameters
    contrast = 1.0;
    brightness = (rand()%101) * (rand()%2 ? 1:-1);
    min_hue = 0;
    max_hue = 105;
    step = 5;
    noise_percentage = (float)(rand()%10 + 1)/10;
    scaling_ratio = (float)(rand()%98 + 1)/100;
    kernel_size = 2 * (rand()%5) + 1;

    std::cout<< "Combinacion: " + std::to_string(combination)<<"\n";
    std::cout<<" Contrast: "<<contrast<<" Brightness: "<<brightness<<"\n";
    std::cout<<" min_hue: "<< std::to_string(min_hue) <<" max_hue: "<< std::to_string(max_hue) <<" step: "<< std::to_string(step) <<"\n";
    std::cout<<" noise_percentage: "<<noise_percentage<<"\n";
    std::cout<<" scaling_ratio (reduction): "<<1 - scaling_ratio<<"\n";
    std::cout<<" kernel_size: "<<kernel_size<<"\n";


    switch (combination){
      case 1:
        // Noise + scaling
        data.SaltPepper(noise_percentage);
        data.in_ = data.out_[0];
        data.Scaling_ROI(scaling_ratio);
        break;
      case 2:
        // Scaling + Gaussian blur
        data.GaussianBlur(kernel_size);
        data.in_ = data.out_[0];
        data.Scaling_ROI(scaling_ratio);
        break;
      case 3:
        // Gaussian blur + hue
        data.GaussianBlur(kernel_size);
        data.in_ = data.out_[0];
        data.out_.clear();
        data.Hue(min_hue, max_hue, step);
        break;
      case 4:
        // Brightness + hue
        data.ContrastBrightness(contrast, brightness);
        data.in_ = data.out_[0];
        data.Hue(min_hue, max_hue, step);
        break;
      case 5:
        // Brightness + noise
        data.ContrastBrightness(contrast, brightness);
        data.in_ = data.out_[0];
        data.SaltPepper(noise_percentage);
        break;
      case 6:
        // Scaling + hue
        data.Scaling_ROI(scaling_ratio);
        data.in_ = data.out_[0];
        data.Hue(min_hue, max_hue, step);
        break;
      case 7:
        // Brightness + hue + blur
        data.ContrastBrightness(contrast, brightness);
        data.in_ = data.out_[0];
        data.GaussianBlur(kernel_size);
        data.in_ = data.out_[0];
        data.Hue(min_hue, max_hue, step);
        break;
      default:
        break;
    }
    for(j=0; j<data.out_.size(); j++){
      cv::imwrite("../../Filtered_imgs/robosub2020"+std::to_string(i + j + 1)+".jpg", data.out_[j]);
    }
    data.out_.clear();
  }
}