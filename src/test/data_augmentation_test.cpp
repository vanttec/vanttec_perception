//------------------------------------------------------------------------------
// @file: data_augmentation_test.cpp
// @created on: March 18th, 2020
// @modified: May 18th, 2020
// @author: Ivana Collado
// @mail:
// @co-author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
// @brief: Use case of data augmentation and tests that it works properly. 
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "../data_augmentation.h"

// MAIN PROGRAM ----------------------------------------------------------------
int main( int argc, char** argv )
{
   cv::Mat in; 
   cv::Mat out;
   // Blurr params
   int kernel_size = 5;
   //ContrastBrightness params 
   double contrast = 1.0;
   int brightness = 80;
   DataAugmentation data;

   //Read input image
   data.Read("../../imgs/lena.png");
   //Apply Gaussian Blur filter
   data.GaussianBlur(kernel_size);
   // Set contrast to 1 and brightness to 80
   data.ContrastBrightness(contrast,brightness);
   //wait for any key to abort
   cv::waitKey(0);
   return 0;
 }
