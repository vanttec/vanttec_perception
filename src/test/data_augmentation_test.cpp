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
    // Input images path
    std::string input_images = "../../imgs/";
    // Processed image path
    std::string filtered_images = "../../Filtered_imgs/";
    // Desired images extension
    std::string extension = ".png";

    int i=0;
    // Read input directory contents
    data.ReadDirectory(input_images, extension, images);
    
    while(!images.empty()){
        // Read each image
        data.ReadImage(input_images+images.front());
        // Apply filters and save if in_ is not NULL
        data.ShowIn();
        // Apply Gaussian Blur filter
        data.GaussianBlur(kernel_size);
        //Change hue
        data.Hue(hue);
        //Add salt and pepper noise
        data.SaltPepper(noise_percentage);
        // Set contrast to 1 and brightness to 80
        data.ContrastBrightness(contrast, brightness);
        //Apply scaling
        data.ScalingROI(ratio, 0);
        data.Save(filtered_images,extension,++i);
        data.ShowOut();
        cv::waitKey(0);
        data.ScalingROI(ratio, 1); 
        data.Save(filtered_images,extension,++i);
        data.ShowOut();
        cv::waitKey(0);
        data.ScalingROI(ratio, 2); 
        data.ShowOut();
        cv::waitKey(0);
        data.Save(filtered_images,extension,++i);
        images.pop();    
    }    
    return 0;
}
