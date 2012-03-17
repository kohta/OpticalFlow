//
//  GrayScaleImageTest.h
//  OpticalFlow
//
//  Created by Kohta Ishikawa on 12/02/16.
//  Copyright (c) 2012年 Kohta Ishikawa. All rights reserved.
//

#ifndef OpticalFlow_GrayScaleImageTest_h
#define OpticalFlow_GrayScaleImageTest_h

#include <opencv2/opencv.hpp>
#include "GrayScaleImage.h"

class GrayScaleImageTest{
public:
    void doTest_xSpatialDiff(std::string imagepath);
    void doTest_ySpatialDiff(std::string imagepath);
    void doTest_temporalDiff(std::string presentimagepath, std::string forwardimagepath);
};

#endif
