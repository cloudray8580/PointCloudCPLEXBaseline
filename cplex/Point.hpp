//
//  Point.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/1.
//  Copyright © 2017年 李喆. All rights reserved.
//
#pragma once
#ifndef Point_hpp
#include"math.h"
#define Point_hpp

#include <stdio.h>

#endif /* Point_hpp */

class Point{
public:
    Point();
    Point(double x, double y, double z);
    Point(double x, double y, double z, int index);
    Point(double x, double y, double z, int index, bool isAvailable);
    Point(double x, double y, double z, int index, bool isAvailable, bool isCenter);
    double x;
    double y;
    double z;
    int index;
    bool isAvailable;
    bool isCenter;
    
    double distanceTo(Point point);
    bool operator < (const Point &p) const
    {
        if (x != p.x){
            return x < p.x;
        }
        if (y != p.y){
            return y < p.y;
        }
        if (z != p.z){
            return z < p.z;
        }
        return false;
    }
    
    bool operator == (const Point &p) const
    {
        return x == p.x && y == p.y && z == p.z;
    }
    
    bool operator != (const Point &p) const
    {
        return x != p.x || y != p.y || z != p.z;
    }
};

Point::Point(){
    x = 0;
    y = 0;
    z = 0;
    index = 0;
    isAvailable = true;
    isCenter = false;
}

Point::Point(double x, double y, double z){
    this->x = x;
    this->y = y;
    this->z = z;
    index = 0;
    isAvailable = true;
    isCenter = false;
}

Point::Point(double x, double y, double z, int index){
    this->x = x;
    this->y = y;
    this->z = z;
    this->index = index;
    isAvailable = true;
    isCenter = false;
}

Point::Point(double x, double y, double z, int index, bool isAvailable){
    this->x = x;
    this->y = y;
    this->z = z;
    this->index = index;
    this->isAvailable = isAvailable;
    isCenter = false;
}

Point::Point(double x, double y, double z, int index, bool isAvailable, bool isCenter){
    this->x = x;
    this->y = y;
    this->z = z;
    this->index = index;
    this->isAvailable = isAvailable;
    this->isCenter = isCenter;
}

double Point::distanceTo(Point point){
    double distanceX = fabs(this->x - point.x);
    double distanceY = fabs(this->y - point.y);
    double distanceZ = fabs(this->z - point.z);
    double distance = sqrt(distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ);
    return distance;
}
