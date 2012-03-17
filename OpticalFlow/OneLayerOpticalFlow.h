//
//  OneLayerOpticalFlow.h
//  OpticalFlow
//
//  Created by Kohta Ishikawa on 12/02/12.
//  Copyright (c) 2012年 Kohta Ishikawa. All rights reserved.
//

#ifndef OpticalFlow_OneLayerOpticalFlow_h
#define OpticalFlow_OneLayerOpticalFlow_h

#include <Eigen/Core>
#include "GrayScaleImage.h"
#include "VectorField2D.h"

class OneLayerOpticalFlow{
public:
    /**
     * constructor in which defines apeature window
     */
    OneLayerOpticalFlow();
    
    /**
     * calculate optical flow from two image frames.
     * non-iterative Lucas-Kanade method is used.
     */
    VectorField2D<double> calcFlow(GrayScaleImage present, GrayScaleImage forward);
    /**
     * !!! currently returns noisy result.
     *     image warping needs to be impremented precisely.
     * 
     * calculate optical flow from two image frames.
     * iterative Lucas-Kanade method is used.
     */
    VectorField2D<double> calcFlowIteratively(GrayScaleImage present, GrayScaleImage forward);
    /**
     * !!! currently returns noisy result.
     *     image warping needs to be impremented precisely.
     * 
     * calculate optical flow from two image frames with initial
     * flow vector field.
     * iterative Lucas-Kanade method is used.
     */
    VectorField2D<double> calcFlowIteratively(GrayScaleImage present, GrayScaleImage forward, VectorField2D<double>& initialFlow);
    
private:
    /**
     * check convergence of flow iteration by means of flow difference
     */
    bool flowConverge(VectorField2D<double>& flowDiff);
    /**
     * check convergence of flow iteration by means of image pixel value difference
     */
    bool flowConverge(GrayScaleImage previous, GrayScaleImage current);
    /**
     * calculate optical flow non-iteratively.
     * it directly uses linear approximation matrix and vector
     * for the sake of reuse of spatial differentials within the iteration.
     */
    VectorField2D<double> calcFlow
        (std::vector< std::vector<Eigen::Matrix2d> >& localInvSpDiffs,
         std::vector< std::vector<Eigen::Vector2d> >& localTemporalDiffs);
    /**
     * Lucas-Kanade optical flow satisfies 2D linear equation
     *   M*(flow) = b
     * this method caluculates matrix part M from 2D spatial differentials.
     */
    std::vector< std::vector<Eigen::Matrix2d> > calcLocalInvSpDiffs(GrayScaleImage& xSpatialDiff, GrayScaleImage& ySpatialDiff);
    /**
     * Lucas-Kanade optical flow satisfies 2D linear equation
     *   M*(flow) = b
     * this method calculates vector part b from 2D spatial differentials
     * and temporal differentials.
     */
    std::vector< std::vector<Eigen::Vector2d> > calcLocalTemporalDiffs(GrayScaleImage& xSpatialDiff, GrayScaleImage& ySpatialDiff, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& temporalDiffs);
    
    /**
     * define apeature window and its weights for calculation of
     * Licas-Kanade equation M*(flow) = b.
     */
    void createLocalApeatureWindow();
    
private:
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> localWindow;
};

#endif
