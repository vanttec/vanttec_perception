//------------------------------------------------------------------------------
// @file: data_augmentation_test.cpp
// @created on: March 18th, 2020
// @modified: July 3rd, 2020
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
    // Set random seed
    srand(time(NULL));
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
    //Scaling_ROI params
    float ratio = 0.7;
    //ContrastBrightness params 
    float contrast = 1.0;
    int brightness = 80;
    // Queue to store the name of input images
    std::queue<std::string> images;
    // Input image path
    std::string input_image = "../../imgs/lena.png";
    // Processed image path
    std::string filtered_images = "../../Filtered_imgs/";
    // Desired images extension
    std::string extension = ".png";
    int i = 0;
    // Read each image
    data.ReadImage(input_image);
    data.ShowIn();
    // Apply Gaussian Blur filter
    data.GaussianBlur(kernel_size);
    data.Save(filtered_images+"filtro_blur",extension); // Save output image
    data.ShowOut(); // Show output image
    cv::waitKey(0);
    //Change hue
    data.Hue(hue);
    data.Save(filtered_images+"filtro_hue",extension); // Save output image
    data.ShowOut(); // Show output image
    cv::waitKey(0);
    //Add salt and pepper noise
    data.SaltPepper(noise_percentage);
    data.Save(filtered_images+"filtro_saltpepper",extension); // Save output image
    data.ShowOut(); // Show output image
    cv::waitKey(0);
    // Set contrast to 1 and brightness to 80
    data.ContrastBrightness(contrast, brightness);
    data.Save(filtered_images+"filtro_brightness",extension); // Save output image
    data.ShowOut(); // Show output image
    cv::waitKey(0);
    //Apply scaling
    data.ScalingROI(ratio, 0); // Upper image
    data.Save(filtered_images+"filtro_scaling_up",extension); // Save output image
    data.ShowOut(); // Show output image
    cv::waitKey(0);
    data.ScalingROI(ratio, 1); // Middle image
    data.Save(filtered_images+"filtro_scaling_mid",extension); // Save output image
    data.ShowOut(); // Show output image
    cv::waitKey(0);
    data.ScalingROI(ratio, 2); // Lower image
    data.Save(filtered_images+"filtro_scaling_low",extension); // Save output image
    data.ShowOut(); // Show output image
    cv::waitKey(0);
    return 0;
}
