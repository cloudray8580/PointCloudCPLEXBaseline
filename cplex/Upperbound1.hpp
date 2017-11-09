//
//  Upperbound1.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/9.
//  Copyright © 2017年 李喆. All rights reserved.
//

#ifndef Upperbound1_hpp
#define Upperbound1_hpp

#include <stdio.h>

#endif /* Upperbound1_hpp */
// using the 1/2max(diam(x), diam(y))
double calculateUpperBound1(const vector<vector<double>> &pointclouds1, const vector<vector<double>> &pointclouds2){
    double upperbound = 0.0;
    long size1 = pointclouds1.size();
    long size2 = pointclouds2.size();
    double diam1 = 0;
    double diam2 = 0;
    double temp = 0;
    for (int i = 0; i < size1; i++){
        for (int j = i; j < size1; j++){
            temp = sqrt((pointclouds1[i][0]-pointclouds1[j][0])*(pointclouds1[i][0]-pointclouds1[j][0]) +(pointclouds1[i][1]-pointclouds1[j][1])*(pointclouds1[i][1]-pointclouds1[j][1]) + (pointclouds1[i][2]-pointclouds1[j][2])*(pointclouds1[i][2]-pointclouds1[j][2]));
            if (temp > diam1)
                diam1 = temp;
        }
    }
    for (int i = 0; i < size2; i++){
        for (int j = i; j < size2; j++){
            temp = sqrt((pointclouds2[i][0]-pointclouds2[j][0])*(pointclouds2[i][0]-pointclouds2[j][0]) +(pointclouds2[i][1]-pointclouds2[j][1])*(pointclouds2[i][1]-pointclouds2[j][1]) + (pointclouds2[i][2]-pointclouds2[j][2])*(pointclouds2[i][2]-pointclouds2[j][2]));
            if (temp > diam2)
                diam2 = temp;
        }
    }
    upperbound = diam1 >= diam2 ? diam1 : diam2;
    upperbound /= 2;
    cout << "=======================================" << endl;
    cout << "upper bound using method1: " << upperbound << endl;
    cout << "=======================================" << endl;
    return upperbound;
}
