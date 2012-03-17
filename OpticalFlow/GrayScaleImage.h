//
//  Image.h
//  OpticalFlow
//
//  Created by Kohta Ishikawa on 12/02/12.
//  Copyright (c) 2012年 Kohta Ishikawa. All rights reserved.
//

#ifndef OpticalFlow_Image_h
#define OpticalFlow_Image_h

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "VectorField2D.h"

class GrayScaleImage{
public:
    /**
     * constructor with image file
     */
    GrayScaleImage(const std::string filePath);
    /**
     * constructor with OpenCV image
     */
    GrayScaleImage(const cv::Mat image);
    /**
     * constructor with grayscale pixel matrix
     */
    GrayScaleImage(const Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> image);
    GrayScaleImage(const GrayScaleImage& image);
    
    /**
     * column number getter
     */
    const int nX() const;
    /**
     * row number getter
     */
    const int nY() const;
    /**
     * accesor to image pixel
     * @param[in] i number of row
     * @param[in] j number of column
     */
    int& at(const int i, const int j);
    /**
     * image pixel getter with homogeneous boundary condition
     * @param[in] i number of row
     * @param[in] j number of column
     */
    const int at_flatBC(const int i, const int j) const;
    /**
     * get image as a OpenCV image
     */
    const cv::Mat getCVImage() const;
    
    /**
     * x direction spatial differentiation (pixel scale)
     * with sobel operator.
     * differectial operator is currently defined in createDiffOperator()
     */
    GrayScaleImage xSpatialDiff();
    /**
     * y direction spatial differentiation (pixel scale)
     * with sobel operator.
     * differectial operator is currently defined in createDiffOperator()
     */
    GrayScaleImage ySpatialDiff();
    /**
     * temporal differentiation
     * @param[in] forward forward(time shifted) image
     * @param[in] timeDiff time interval of forward and the image
     */
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> temporalDiff( GrayScaleImage forward, const double timeDiff);
    /**
     * image warping with pixel movements(displacement vector field)
     * !!! currently irrelevant!
     */
    void warp(VectorField2D<double>& diff);
private:
    /**
     * differential operator definition
     */
    void createDiffOperator();
private:
    Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> xDiffOperator;
    Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> yDiffOperator;
    Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> image;
};

#endif
