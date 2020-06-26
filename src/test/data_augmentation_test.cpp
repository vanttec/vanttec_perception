//------------------------------------------------------------------------------
// @file: data_augmentation_test.cpp
// @created on: March 18th, 2020
// @modified: May 18th, 2020
// @author: Ivana Collado
// @mail: ivanacollado@gmail.com
// @co-author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
// @brief: Use case of data augmentation and tests that it works properly. 
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "../include/data_augmentation.h"

// MAIN PROGRAM ----------------------------------------------------------------
int main( int argc, char** argv ){
    // Class object 
    DataAugmentation data;
    //Blur param
    int kernel_size = 5;
    //Hue params
    int min_hue = 0;
    int max_hue = 105;
    int step = 5;
    //Hue param
    int hue = rand()% (max_hue - min_hue + 1) + min_hue;
    //Salt and pepper param
    float noise_percentage = 0.2;
    //Scaling_ROI param
    float ratio = 0.5;
    //ContrastBrightness params 
    float contrast = 1.0;
    int brightness = 80;
    // Vector to store desired images
    std::vector<std::string> images;
    // Input image path
    std::string input_image = "../../imgs/lena.png";
    // Processed image path
    std::string filtered_images = "../../Filtered_imgs/";

    //Read input image
    data.Read(input_image);
    // Apply Gaussian Blur filter
    data.GaussianBlur(kernel_size);
    //Change hue
    data.Hue(hue);
    //Add salt and pepper noise
    data.SaltPepper(noise_percentage);
    //Apply scaling
    data.ScalingROI(ratio); 
    // Set contrast to 1 and brightness to 80
    data.ContrastBrightness(contrast, brightness);
    // Save last method
    data.Save(filtered_images);
    //wait for any key to abort
    cv::waitKey(0);
    return 0;
}
