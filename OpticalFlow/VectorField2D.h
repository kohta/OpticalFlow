//
//  VectorField.h
//  OpticalFlow
//
//  Created by Kohta Ishikawa on 12/02/12.
//  Copyright (c) 2012年 Kohta Ishikawa. All rights reserved.
//

#ifndef OpticalFlow_VectorField2D_h
#define OpticalFlow_VectorField2D_h

#include <vector>

/**
 * 2 dimensional vector field
 * template class indicates type of vector element value
 */
template <class T>
class VectorField2D{
public:
    /**
     * constructor with nums of rows and columns
     */
    VectorField2D(const int ny, const int nx);
    VectorField2D(const VectorField2D& vf);
    VectorField2D& operator=(const VectorField2D& vf);
    
    /**
     * add opration for each vector on the field
     */
    VectorField2D operator+(const VectorField2D& vf);
    
    /**
     * row getter (for fast access)
     */
    const std::vector< std::vector<T> >& row(const int i) const;
    /**
     * 
     */
    std::vector<T>& at(const int i, const int j);
    
    /**
     * column number getter 
     */
    const int nX() const;
    /**
     * row number getter 
     */
    const int nY() const;
    
private:
    int nx;
    int ny;
    std::vector< std::vector< std::vector<T> > > field; 
};

template <class T>
VectorField2D<T>::VectorField2D(const int ny, const int nx)
{
    field = std::vector< std::vector< std::vector<T> > >
              (ny, std::vector< std::vector<T> >
                    (nx, std::vector<T>(2,T()))
               );
    this->nx = nx;
    this->ny = ny;
}

template <class T>
VectorField2D<T>::VectorField2D(const VectorField2D& vf)
{
    nx = vf.nx;
    ny = vf.ny;
    for(int i=0; i<ny; i++){
        this->field.at(i) = vf.field.at(i);
    }
}

template <class T>
VectorField2D<T>& VectorField2D<T>::operator=(const VectorField2D<T>& vf)
{
    nx = vf.nx;
    ny = vf.ny;
    for(int i=0; i<ny; i++){
        this->field.at(i) = vf.field.at(i);
    }
    return (*this);
}

template <class T>
VectorField2D<T> VectorField2D<T>::operator+(const VectorField2D<T>& vf)
{
    VectorField2D<T> vfTmp(this->ny, this->nx);
    for(int i=0; i<ny; i++){
        for(int j=0; j<nx; j++){
            vfTmp.field[i][j][0] = this->field[i][j][0] + vf.field[i][j][0];
            vfTmp.field[i][j][1] = this->field[i][j][1] + vf.field[i][j][1];
        }
    }
    return vfTmp;
}

template <class T>
const std::vector< std::vector<T> >& VectorField2D<T>::row(const int i)  const
{
    return field.at(i);
}

template <class T>
std::vector<T>& VectorField2D<T>::at(const int i, const int j)
{
    return field.at(i).at(j);
}

template <class T>
const int VectorField2D<T>::nX() const
{
    return nx;
}


template <class T>
const int VectorField2D<T>::nY() const
{
    return ny;
}

#endif
