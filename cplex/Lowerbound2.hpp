//
//  Lowerbound2.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/9.
//  Copyright © 2017年 李喆. All rights reserved.
//

#ifndef Lowerbound2_hpp
#define Lowerbound2_hpp

#include <stdio.h>

#endif /* Lowerbound2_hpp */

// 1/2 inf R SUP (x,y) in R |eccx(X) - eccy(Y)|
double calculateLowerBound2(const vector<vector<double>> &pointclouds1, const vector<vector<double>> &pointclouds2){
    
    double lowerbound = 0;
    vector<double> ecc1 = calculateEccentricity(pointclouds1);
    vector<double> ecc2 = calculateEccentricity(pointclouds2);
    
    long size1 = ecc1.size();
    long size2 = ecc2.size(); // size 2 should equal to size 1 !
    
    double tempdistance;
    vector<vector<double>> cost;
    
    // create the cost matrix
    for (int i = 0; i < size1; i++){
        for (int j = 0; j < size2; j++){
            tempdistance = fabs(ecc1[i] - ecc2[j]);
            cost[i][j] = tempdistance;
        }
    }
    
    double determinant = calculateDeterminantTest(cost, size1);
    if (determinant != 0){
        cout << "There is a perfect match " << endl;
    }
    
    return lowerbound;
}

vector<double> calculateEccentricity(const vector<vector<double>> &pointclouds){
    
    vector<double> ecc;
    double tempdistance = 0;
    double max = 0;
    
    long size = pointclouds.size();
    for (int i = 0; i < size; i++){
        max = 0;
        for (int j = 0; j < size; j++){
            tempdistance = sqrt((pointclouds[i][0] - pointclouds[j][0])*(pointclouds[i][0] - pointclouds[j][0]) + (pointclouds[i][1] - pointclouds[j][1])*(pointclouds[i][1] - pointclouds[j][1]) + (pointclouds[i][2] - pointclouds[j][2])*(pointclouds[i][2] - pointclouds[j][2]));
            if (tempdistance > max){
                max = tempdistance;
            }
        }
        ecc.push_back(max);
    }
    
    return ecc;
}

// this is just used for test, do not use this one to calculate determinant
double calculateDeterminantTest(vector<vector<double>> matrix, int size){
    double det=0;
    vector<vector<double>> temp = matrix;
    int p, h, k, i, j;
    if(size==1) {
        return matrix[0][0];
    } else if(size==2) {
        det=(matrix[0][0]*matrix[1][1]-matrix[0][1]*matrix[1][0]);
        return det;
    } else {
        for(p=0;p<size;p++) {
            h = 0;
            k = 0;
            for(i=1;i<size;i++) {
                for( j=0;j<size;j++) {
                    if(j==p) {
                        continue;
                    }
                    temp[h][k] = matrix[i][j];
                    k++;
                    if(k==size-1) {
                        h++;
                        k = 0;
                    }
                }
            }
            det=det+matrix[0][p]*pow(-1,p)*calculateDeterminantTest(temp,size-1);
        }
        return det;
    }
}
