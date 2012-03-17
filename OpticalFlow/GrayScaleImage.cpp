//
//  Image.cpp
//  OpticalFlow
//
//  Created by Kohta Ishikawa on 12/02/12.
//  Copyright (c) 2012年 Kohta Ishikawa. All rights reserved.
//

#include <iostream>
#include <opencv2/core/eigen.hpp>
#include "GrayScaleImage.h"


GrayScaleImage::GrayScaleImage(const std::string filePath)
{
    createDiffOperator();
    cv::Mat cvImage = cv::imread(filePath,0);
    cv::cv2eigen(cvImage,image);
}

GrayScaleImage::GrayScaleImage(const cv::Mat image)
{
    createDiffOperator();
    cv::cv2eigen(image,this->image);
}

GrayScaleImage::GrayScaleImage(Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> image)
{
    createDiffOperator();
    this->image = image;
}

GrayScaleImage::GrayScaleImage(const GrayScaleImage& image)
{
    createDiffOperator();
    this->image = image.image;
}

const int GrayScaleImage::nX() const
{
    return image.cols();
}

const int GrayScaleImage::nY() const
{
    return image.rows();
}

int& GrayScaleImage::at(const int i, const int j)
{
    return image(i,j);
}

const int GrayScaleImage::at_flatBC(int i, int j) const
{
    if(i < 0){
        i = 0;
    }else if(i >= image.rows()){
        i = image.rows() - 1;
    }
    if(j < 0){
        j = 0;
    }else if(j >= image.cols()){
        j = image.cols() - 1;
    }
    return image(i,j);
}

const cv::Mat GrayScaleImage::getCVImage() const
{
    cv::Mat cvImage;
    cv::eigen2cv(image, cvImage);
    return cvImage;
}

GrayScaleImage GrayScaleImage::xSpatialDiff()
{
    Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> xDiff(image.rows(),image.cols());
    for(int i=0; i<image.rows(); i++){
        for(int j=0; j<image.cols();j++){
            xDiff(i,j) = 0;
            for(int ix=0; ix<xDiffOperator.cols();ix++){
                for(int iy=0; iy<xDiffOperator.rows(); iy++){
                    xDiff(i,j) += xDiffOperator(iy,ix)*this->at_flatBC(i-1+iy, j-1+ix)/4;
                }
            }
        }
    }
    return GrayScaleImage(xDiff);
}

GrayScaleImage GrayScaleImage::ySpatialDiff()
{
    Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> yDiff(image.rows(),image.cols());
    for(int i=0; i<image.rows(); i++){
        for(int j=0; j<image.cols();j++){
            yDiff(i,j) = 0;
            for(int ix=0; ix<yDiffOperator.cols();ix++){
                for(int iy=0; iy<yDiffOperator.rows(); iy++){
                    yDiff(i,j) += yDiffOperator(iy,ix)*this->at_flatBC(i-1+iy, j-1+ix)/4;
                }
            }
        }
    }
    return GrayScaleImage(yDiff);
}

Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> 
GrayScaleImage::temporalDiff(GrayScaleImage forward, const double timeDiff)
{
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> tDiff(image.rows(),image.cols());
    for(int i=0; i<image.rows(); i++){
        for(int j=0; j<image.cols();j++){
            tDiff(i,j) = (forward.at(i,j) - this->at(i,j))/timeDiff;
        }
    }
    return tDiff;
}

void GrayScaleImage::warp(VectorField2D<double>& diff)
{
    Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic> currentImage;
    currentImage = image;
    for(int i=0; i<diff.nY(); i++){
        for(int j=0; j<diff.nX(); j++){
            int diffi = i + (int)diff.at(i,j)[1];
            int diffj = j + (int)diff.at(i,j)[0];
            if(diffi >= diff.nY()){
                diffi = diff.nY()-1;
            }else if(diffi < 0){
                diffi = 0;
            }
            if(diffj >= diff.nX()){
                diffj = diff.nX()-1;
            }else if(diffj < 0){
                diffj = 0;
            }
            double delta = sqrt(diff.at(i,j)[0]*diff.at(i,j)[0] + diff.at(i,j)[1]*diff.at(i,j)[1]);
            image(i,j) = currentImage(diffi,diffj);
        }
    }
    return;
}

void GrayScaleImage::createDiffOperator()
{
    xDiffOperator = Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic>(3,3);
    xDiffOperator << -1,0,1,
                     -2,0,2,
                     -1,0,1;
    yDiffOperator = Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic>(3,3);
    yDiffOperator << -1,-2,-1,
                      0, 0, 0,
                      1, 2, 1;
}