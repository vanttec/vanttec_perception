//------------------------------------------------------------------------------
// @file: data_augmentation_test.cpp
// @created on: March 18th, 2019
// @author: Ivana Collado
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
   int kernel_size=5;
   double alpha=1.0;
   int beta=80;
   DataAugmentation data;

   //Read input image
   data.Read("../../imgs/lena.png");
   //Apply Gaussian Blur filter
   data.GaussianBlur(kernel_size);
   // Set contrast to 1 and brightness to 80
   data.ContrastBrightness(alpha,beta);
   //wait for any key to abort
   cv::waitKey(0);
   return 0;
 }
