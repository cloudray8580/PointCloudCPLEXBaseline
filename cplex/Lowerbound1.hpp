//
//  Lowerbound1.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/9.
//  Copyright © 2017年 李喆. All rights reserved.
//

#ifndef Lowerbound1_hpp
#define Lowerbound1_hpp

#include <stdio.h>

#endif /* Lowerbound1_hpp */

// 1/2 max ( |diam(x) - diam(y)| , |rad(x) - rad(y)| )
double calculateLowerBound1(const vector<vector<double>> &pointclouds1, const vector<vector<double>> &pointclouds2){
    
    time_t start,stop;
    start = time(NULL);
    
    double lowerbound = 0;
    
    long size1 = pointclouds1.size();
    long size2 = pointclouds2.size();
    
    double diam1 = 0;
    double diam2 = 0;
    double rad1 = 0;
    double rad2 = 0;
    double max = 0;
    double temp = 0;
    bool firsttry = true;
    
    for (int i = 0; i < size1; i++){
        for (int j = 0; j < size1; j++){ // when calculation radius, you need to start from 0, but for diameter, you can start from j = i.
            temp = sqrt((pointclouds1[i][0]-pointclouds1[j][0])*(pointclouds1[i][0]-pointclouds1[j][0]) +(pointclouds1[i][1]-pointclouds1[j][1])*(pointclouds1[i][1]-pointclouds1[j][1]) + (pointclouds1[i][2]-pointclouds1[j][2])*(pointclouds1[i][2]-pointclouds1[j][2]));
            if (temp > diam1)
                diam1 = temp;
            if (temp > max){
                max = temp;
            }
        }
        if (firsttry){
            rad1 = max;
            firsttry = false;
        }
        if (max < rad1){
            rad1 = max;
        }
        max = 0;
    }
    
    firsttry = true;
    max = 0;
    
    for (int i = 0; i < size2; i++){
        for (int j = 0; j < size2; j++){
            temp = sqrt((pointclouds2[i][0]-pointclouds2[j][0])*(pointclouds2[i][0]-pointclouds2[j][0]) +(pointclouds2[i][1]-pointclouds2[j][1])*(pointclouds2[i][1]-pointclouds2[j][1]) + (pointclouds2[i][2]-pointclouds2[j][2])*(pointclouds2[i][2]-pointclouds2[j][2]));
            if (temp > diam2)
                diam2 = temp;
            if (temp > max){
                max = temp;
            }
        }
        if (firsttry){
            rad2 = max;
            firsttry = false;
        }
        if (max < rad2){
            rad2 = max;
        }
        max = 0;
    }
    
    double part1 = fabs(diam1 - diam2);
    double part2 = fabs(rad1 - rad2);
    
    lowerbound = part1 > part2 ? part1 : part2;
    lowerbound /= 2;
    stop = time(NULL);
    cout << "=================================================" << endl;
    cout << "  Lowerbound1 : " << lowerbound << "   time : " << stop-start << endl;
    cout << "  diam1 : " << diam1 << "      diam2 : " << diam2 << endl;
    cout << "  rad1 : " << rad1 << "    rad2 : " << rad2 << endl;
    cout << "=================================================" << endl;
    
    return lowerbound ;
}
