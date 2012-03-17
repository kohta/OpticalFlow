//
//  GrayScaleImageTest.cpp
//  OpticalFlow
//
//  Created by Kohta Ishikawa on 12/02/16.
//  Copyright (c) 2012年 Kohta Ishikawa. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "GrayScaleImageTest.h"

void GrayScaleImageTest::doTest_xSpatialDiff(std::string imagepath)
{
    GrayScaleImage image(imagepath);
    GrayScaleImage xsd = image.xSpatialDiff();
    cv::Mat result = xsd.getCVImage();
    cv::imwrite(imagepath + "_xdiff.png",result);
}

void GrayScaleImageTest::doTest_ySpatialDiff(std::string imagepath)
{
    GrayScaleImage image(imagepath);
    GrayScaleImage ysd = image.ySpatialDiff();
    cv::Mat result = ysd.getCVImage();
    cv::imwrite(imagepath + "_ydiff.png",result);
}

void GrayScaleImageTest::doTest_temporalDiff(std::string presentimagepath, std::string forwardimagepath)
{
    GrayScaleImage present(presentimagepath);
    GrayScaleImage forward(forwardimagepath);
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> td = present.temporalDiff(forward,1);
    cv::Mat cvtd;
    cv::eigen2cv(td,cvtd);
    imwrite(presentimagepath + "_tdiff.png",cvtd);
}