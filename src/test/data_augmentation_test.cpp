//------------------------------------------------------------------------------
// @file: data_augmentation_test.cpp
// @created on: March 18th, 2019
// @author: Ivana Collado
// 
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
   u_char min_hue=0,max_hue=105,step=5;
   DataAugmentation data;

   //Read input image
   data.Read("../../imgs/lena.png");
   //Apply average filter
   data.AverageFilter(kernel_size);
   //Change hue from min_hue to max_hue in steps
   data.Hue(min_hue,max_hue,step);
   //wait for any key to abort
   cv::waitKey(0);
   return 0;
 }
