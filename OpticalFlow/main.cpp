//
//  main.cpp
//  OpticalFlow
//
//  Created by Kohta Ishikawa on 12/02/12.
//  Copyright (c) 2012年 Kohta Ishikawa. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "GrayScaleImage.h"
#include "VectorField2D.h"
#include "OneLayerOpticalFlow.h"

//test
#include "GrayScaleImageTest.h"

int main (int argc, const char * argv[])
{
    if(argc < 4){
        std::cout << "execute along with two image file paths." << std::endl;
    }else if(argc > 4){
        std::cout << "too many arguments." << std::endl;
    }
    FILE* fp;
    if((fp = fopen(argv[1],"r")) == NULL){
        std::cout << "input file 1 does not exist." << std::endl;
        return 0;
    }else{
        fclose(fp);
    }
    if((fp = fopen(argv[2],"r")) == NULL){
        std::cout << "input file 2 does not exist." << std::endl;
        return 0;
    }else{
        fclose(fp);
    }
    
    std::string present(argv[1]);
    std::string forward(argv[2]);
    GrayScaleImage presentImage(present);
    GrayScaleImage forwardImage(forward);
    
    OneLayerOpticalFlow* of = new OneLayerOpticalFlow();
    
    VectorField2D<double> opticalFlow = of->calcFlow(presentImage,forwardImage);
    //VectorField2D<double> opticalFlow = of->calcFlowIteratively(presentImage, forwardImage);
    
    char* name = "result";
    cv::Mat cvPresentImage = cv::imread(present);
    cv::namedWindow(name, CV_WINDOW_AUTOSIZE);
    for(int i=0; i<opticalFlow.nX(); i++){
        for(int j=0; j<opticalFlow.nY(); j++){
            if(i%5 == 0 && j%5 == 0){
                cv::line(cvPresentImage, cv::Point(i,j), cv::Point(i+opticalFlow.at(j,i)[0], j+opticalFlow.at(j,i)[1]),cv::Scalar(0,0,200),1,CV_AA);
            }
        }
    }
    cv::imwrite(argv[3],cvPresentImage);    
    return 0;
}

