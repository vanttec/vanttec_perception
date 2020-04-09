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
   DataAugmentation data;

   //Read input image
   data.Read("../../imgs/lena.png");
   //Apply average filter
   data.AverageFilter(kernel_size);
   //Rotate -45 degrees
   data.Rotation(-45);
   //Change hue from 90 to 120 in steps of 5
   data.Hue(90,120,5);
   //wait for any key to abort
   cv::waitKey(0);
   return 0;
 }
