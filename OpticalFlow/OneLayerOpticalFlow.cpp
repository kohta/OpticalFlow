//
//  OneLayerOpticalFlow.cpp
//  OpticalFlow
//
//  Created by Kohta Ishikawa on 12/02/13.
//  Copyright (c) 2012年 Kohta Ishikawa. All rights reserved.
//

#include <iostream>
#include "OneLayerOpticalFlow.h"

OneLayerOpticalFlow::OneLayerOpticalFlow()
{
    createLocalApeatureWindow();
}

VectorField2D<double> OneLayerOpticalFlow::calcFlow
(GrayScaleImage present, GrayScaleImage forward)
{
    //calculate Spatial Differenciates
    GrayScaleImage xSpatialDiff = present.xSpatialDiff();
    GrayScaleImage ySpatialDiff = present.ySpatialDiff();
    std::vector< std::vector<Eigen::Matrix2d> > 
    localInvSpDiffs = calcLocalInvSpDiffs(xSpatialDiff,ySpatialDiff);
    
    //calculate flow
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> 
    temporalDiff = present.temporalDiff(forward, 1);
    std::vector< std::vector<Eigen::Vector2d> > localTemporalDiffs
    = calcLocalTemporalDiffs(xSpatialDiff, ySpatialDiff, temporalDiff);
    return calcFlow(localInvSpDiffs, localTemporalDiffs);
}

VectorField2D<double> OneLayerOpticalFlow::calcFlowIteratively
(GrayScaleImage present, GrayScaleImage forward)
{
    //calculate Spatial Differenciates
    GrayScaleImage xSpatialDiff = present.xSpatialDiff();
    GrayScaleImage ySpatialDiff = present.ySpatialDiff();
    std::vector< std::vector<Eigen::Matrix2d> > 
        localInvSpDiffs = calcLocalInvSpDiffs(xSpatialDiff,ySpatialDiff);
    
    //calculate first flow
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> 
        temporalDiff = present.temporalDiff(forward, 1);
    std::vector< std::vector<Eigen::Vector2d> > localTemporalDiffs
        = calcLocalTemporalDiffs(xSpatialDiff, ySpatialDiff, temporalDiff);
    VectorField2D<double> flow = calcFlow(localInvSpDiffs, localTemporalDiffs);
    
    //iteration
    GrayScaleImage current = present;
    while(true){
        //calculate flows until converge
        //GrayScaleImage current = present;
        GrayScaleImage previous = current;
        current.warp(flow);
        //if(flowConverge(current, present)){
        if(flowConverge(current, previous)){
            break;
        }
        //temporalDiff = present.temporalDiff(forward, 1);
        temporalDiff = current.temporalDiff(forward, 1);
        localTemporalDiffs = calcLocalTemporalDiffs(xSpatialDiff, ySpatialDiff, temporalDiff);
        //VectorField2D<double> nextFlow = calcFlow(localInvSpDiffs, localTemporalDiffs);
        //flow = flow + nextFlow;
        flow = calcFlow(localInvSpDiffs, localTemporalDiffs);
    }
    
    return flow;
}

VectorField2D<double> OneLayerOpticalFlow::calcFlowIteratively
(GrayScaleImage present, GrayScaleImage forward, VectorField2D<double>& initialFlow)
{
    present.warp(initialFlow);
    return calcFlowIteratively(present,forward);
}

bool OneLayerOpticalFlow::flowConverge(VectorField2D<double>& flowDiff)
{
    double epsilon = 1.5;
    bool converge = true;
    for(int i=0; i<flowDiff.nY(); i++){
        for(int j=0; j<flowDiff.nX(); j++){
            double flowDiff_ijx = flowDiff.at(i,j)[1];
            double flowDiff_ijy = flowDiff.at(i,j)[0];
            double vij = std::sqrt(flowDiff_ijx*flowDiff_ijx + flowDiff_ijy*flowDiff_ijy);
            if(vij >= epsilon){
                converge = false;
                break;
            }
        }
    }
    return converge;
}

bool OneLayerOpticalFlow::flowConverge(GrayScaleImage previous, GrayScaleImage current)
{
    int nx = previous.nX();
    int ny = previous.nY();
    double obj = 0.0;
    for(int i=0; i<ny; i++){
        for(int j=0; j<nx; j++){
            obj += (current.at(i,j) - previous.at(i,j)) 
                    * (current.at(i,j) - previous.at(i,j));
        }
    }
    obj /= nx*ny;
    std::cout << obj << std::endl;
    if(obj < 10){
        return true;
    }
    return false;
}

VectorField2D<double> OneLayerOpticalFlow::calcFlow
(std::vector< std::vector<Eigen::Matrix2d> >& localInvSpDiffs,
 std::vector< std::vector<Eigen::Vector2d> >& localTemporalDiffs)
{
    int ny = localTemporalDiffs.size();
    int nx = localTemporalDiffs.at(0).size();
    VectorField2D<double> flow(ny, nx);
    for(int i=0; i<ny; i++){
        for(int j=0; j<nx; j++){
            Eigen::Vector2d ve = localInvSpDiffs.at(i).at(j) 
                                    * localTemporalDiffs.at(i).at(j);
            std::vector<double> v(2);
            v[0] = ve(0);
            v[1] = ve(1);
            flow.at(i,j) = v;
        }
    }
    return flow;
}

std::vector< std::vector<Eigen::Matrix2d> > 
OneLayerOpticalFlow::calcLocalInvSpDiffs
(GrayScaleImage& xSpatialDiff, GrayScaleImage& ySpatialDiff)
{
    int ny = xSpatialDiff.nY();
    int nx = xSpatialDiff.nX();
    std::vector< std::vector<Eigen::Matrix2d> > spDiffs(ny);
    for(int i=0; i<ny; i++){
        spDiffs[i] = std::vector<Eigen::Matrix2d>(nx);
        for(int j=0; j<nx; j++){
            Eigen::Matrix2d mat = Eigen::Matrix2d::Zero(2,2);
            for(int iy=0; iy<3; iy++){
                for(int ix=0; ix<3; ix++){
                    int yref = i-1+iy;
                    int xref = j-1+ix;
                    if(yref<0){
                        yref = 0; //境界条件
                    }else if(yref>=ny){
                        yref = ny-1;
                    }
                    if(xref<0){
                        xref = 0; //境界条件
                    }else if(xref>=nx){
                        xref = nx-1;
                    }
                    mat(0,0) += localWindow(iy,ix)
                     *xSpatialDiff.at(yref,xref)*xSpatialDiff.at(yref,xref);
                    mat(0,1) += localWindow(iy,ix)
                     *xSpatialDiff.at(yref,xref)*ySpatialDiff.at(yref,xref);
                    mat(1,0) = mat(0,1);
                    mat(1,1) += localWindow(iy,ix)
                     *ySpatialDiff.at(yref,xref)*ySpatialDiff.at(yref,xref);
                }
            }
            double d = mat(0,0)*mat(1,1) - mat(0,1)*mat(1,0);
            if(d < 1e-12){
                //zero matrix when inverse matrix unavailable
                //(this leads zero flow)
                spDiffs[i][j] = mat.Zero();
            }else{
                Eigen::Matrix2d matInv;
                matInv(0,0) = mat(1,1)/d;
                matInv(0,1) = -mat(0,1)/d;
                matInv(1,0) = -mat(1,0)/d;
                matInv(1,1) = mat(0,0)/d;
                spDiffs[i][j] = matInv;
            }
        }
    }
    return spDiffs;
}

std::vector< std::vector<Eigen::Vector2d> > 
OneLayerOpticalFlow::calcLocalTemporalDiffs
(GrayScaleImage& xSpatialDiff, GrayScaleImage& ySpatialDiff,
 Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& temporalDiff)
{
    int ny = xSpatialDiff.nY();
    int nx = xSpatialDiff.nX();
    std::vector< std::vector<Eigen::Vector2d> > tmpDiffs(ny);
    for(int i=0; i<ny; i++){
        std::vector<Eigen::Vector2d> diff_i(nx);
        for(int j=0; j<nx; j++){
            Eigen::Vector2d vec = Eigen::Vector2d::Zero(2);
            for(int iy=0; iy<3; iy++){
                for(int ix=0; ix<3; ix++){
                    int yref = i-1+iy;
                    int xref = j-1+ix;
                    if(yref<0){
                        yref = 0;
                    }else if(yref>=ny){
                        yref = ny-1;
                    }
                    if(xref<0){
                        xref = 0;
                    }else if(xref>=nx){
                        xref = nx-1;
                    }
                    vec(0) -= localWindow(iy,ix)
                     *xSpatialDiff.at(yref,xref)*temporalDiff(yref,xref);
                    vec(1) -= localWindow(iy,ix)
                     *ySpatialDiff.at(yref,xref)*temporalDiff(yref,xref);
                }
            }
            diff_i.at(j) = vec;
        }
        tmpDiffs.at(i) = diff_i;
    }
    return tmpDiffs;
}

void OneLayerOpticalFlow::createLocalApeatureWindow()
{
    //3*3 gaussian
    localWindow = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>(3,3);
    localWindow << 0.07511361, 0.1238414, 0.07511361,
                   0.1238414,  0.20418,   0.1238414,
                   0.07511361, 0.1238414, 0.07511361;
}
