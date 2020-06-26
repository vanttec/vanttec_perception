//------------------------------------------------------------------------------
// @file: data_augmentation.cpp
// @created on: June 25th, 2020
// @modified: June 29th, 2020
// @co-author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
// @brief: Use case of data augmentation and final implementation. There are
//         seven possible combinations for the input images.
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "include/data_augmentation.h"

// MAIN PROGRAM ----------------------------------------------------------------
int main( int argc, char** argv ){
    // Instance of DataAugmentation class
    DataAugmentation data;
    // Choose which filter combination will be applied
    int combination = 0;
    // For the Gaussian blur filter
    int kernel_size = 0;
    // For the ContrastBrightness filter
    int brightness = 0;
    // For the ContrastBrightness filter
    double contrast = 1.0;
    // Maximum acceptable value for the hue filter
    int max_hue = 105;
    // Minimum acceptable value for the hue filter
    int min_hue = 0;
    // Hue value in the min-max range
    int hue = 0;
    // Maximum acceptable noise value for the SaltPepper filter
    int max_noise = 4;
    // Minimum acceptable noise value for the SaltPepper filter
    int min_noise = 1;
    //Noise value for the SaltPepper filter
    float noise_percentage = 0.0;
    // Maximum acceptable scaling value for the ScalingROI filter  
    int max_scaling = 95;
    // Minimum acceptable scaling value for the ScalingROI filter
    int min_scaling = 60;
    // Scaling value for the ScalingROI filter
    float scaling_ratio = 0.0;
    // Vector to store desired images
    std::vector<std::string> images = {};
    // Images to process path
    std::string imgs_path = "../../imgs/";
    // Save processed images path
    std::string save_path = "../../Filtered_imgs/robosub2020_ ";

    // Read directory contents (png images only)
    data.ReadDirectory(imgs_path,images);

    for(int i=0; i<images.size(); i++){
        // Read a single image
        data.Read(imgs_path + images[i]);
        // Define parameters
        // Random between 1 and 7
        combination = rand()%7 + 1; 
        // Random between -100 and 100
        brightness = (rand()%101) * (rand()%2 ? 1:-1); 
        // Random between 0 and 105
        hue = rand()% (max_hue - min_hue + 1) + min_hue; 
        // Random between 0 and 0.4
        noise_percentage = (float) (rand()% max_noise + min_noise)/10; 
        // Random between 0.6 and 0.95
        scaling_ratio = (float) ((rand()% (max_scaling - min_scaling + 1)) +
                                                            min_scaling)/100;
        // Random odd number between 1 and 25
        kernel_size = 2 * (rand()%13) + 1;

        // std::cout<<"Combination: "+std::to_string(combination)<<"\n";
        // std::cout<<"Contrast: "<<contrast<<" Brightness: "<<brightness<<"\n";
        // std::cout<<"min_hue: "<< min_hue 
        //         <<" max_hue: "<< max_hue
        //         <<" hue: "<< hue <<"\n";
        // std::cout<<"noise_percentage: "<<noise_percentage<<"\n";
        // std::cout<<"scaling_ratio (reduction): "<<1-scaling_ratio<<"\n";
        // std::cout<<"kernel_size: "<<kernel_size<<"\n";

        switch (combination){
        case 1:
            // Noise + scaling
            data.SaltPepper(noise_percentage);
            data.SetIn(data.GetOut().back());
            data.PopBack();
            data.ScalingROI(scaling_ratio);
            break;
        case 2:
            // Scaling + Gaussian blur
            data.GaussianBlur(kernel_size);
            data.SetIn(data.GetOut().back());
            data.PopBack();
            data.ScalingROI(scaling_ratio);
            break;
        case 3:
            // Gaussian blur + hue
            data.GaussianBlur(kernel_size);
            data.SetIn(data.GetOut().back());
            data.PopBack();
            data.Hue(hue);
            break;
        case 4:
            // Brightness + hue
            data.ContrastBrightness(contrast, brightness);
            data.SetIn(data.GetOut().back());
            data.PopBack();
            data.Hue(hue);
            break;
        case 5:
            // Brightness + noise
            data.ContrastBrightness(contrast, brightness);
            data.SetIn(data.GetOut().back());
            data.PopBack();
            data.SaltPepper(noise_percentage);
            break;
        case 6:
            // Scaling + hue
            data.Hue(hue);
            data.SetIn(data.GetOut().back());
            data.PopBack();
            data.ScalingROI(scaling_ratio);
            break;
        case 7:
            // Brightness + hue + blur
            data.ContrastBrightness(contrast, brightness);
            data.SetIn(data.GetOut().back());
            data.PopBack();
            data.GaussianBlur(kernel_size);
            data.SetIn(data.GetOut().back());
            data.PopBack();
            data.Hue(hue);
            break;
        default:
            break;
        }
    }
    data.Save(save_path);
    return 0;
}