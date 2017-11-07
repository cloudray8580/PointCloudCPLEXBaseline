//
//  PointCloudsCalculation.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/2.
//  Copyright © 2017年 李喆. All rights reserved.
//
#pragma once
#ifndef PointCloudsCalculation_hpp
#define PointCloudsCalculation_hpp

#include <stdio.h>
#include <ilcplex/ilocplex.h>
#include "PointCloud.hpp"
#endif /* PointCloudsCalculation_hpp */

class PointCloudsCalculation{
public:
    struct LB_UB{
        double lowerbound;
        double upperbound;
    };
    
    static LB_UB LowerboundAndUpperbound_kmedoids(int k, PointCloud pointcloud1, PointCloud pointcloud2);
    
    static double GHDistance_CPLEX(PointCloud pointcloud1, PointCloud pointcloud2);
    
    static double myCplexGHDSolve(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2);
    static double myCalculateDistortion(const int i, const int j, const int k, const int l, const vector<vector<double>> &pointclouds1,
                                      const vector<vector<double>> &pointclouds2);
};

PointCloudsCalculation::LB_UB PointCloudsCalculation::LowerboundAndUpperbound_kmedoids(int k, PointCloud pointcloud1, PointCloud pointcloud2){
    double lowerbound;
    double upperbound;
    
    double maxRadius1;
    double maxRadius2;
    
    double GHDistance;
    
    PointCloud::clusterResult result1 = pointcloud1.runKmedoids(k);
    cout << "=============" << endl;
    PointCloud::clusterResult result2 = pointcloud2.runKmedoids(k);
    
    for(auto it = result1.clusters.begin(); it != result1.clusters.end(); it++){
        cout << "center: " << endl;
        cout << it->first.x << " " << it->first.y << " " << it->first.z << endl;
        cout << "points: " << endl;
        for (int i = 0; i < it->second.size(); i++){
            cout << it->second[i].x << " " << it->second[i].y << " " << it->second[i].z << endl;
        }
    }
    
    
    maxRadius1 = pointcloud1.calculateClusterMaxRadius(result1.clusters);
    maxRadius2 = pointcloud2.calculateClusterMaxRadius(result2.clusters);
    
    GHDistance = GHDistance_CPLEX(result1.centers, result2.centers);
    
    lowerbound = GHDistance - 2*maxRadius1 - 2*maxRadius2;
    upperbound = GHDistance + 2*maxRadius1 + 2*maxRadius2;
    
    LB_UB bounds;
    bounds.lowerbound = lowerbound;
    bounds.upperbound = upperbound;
    return bounds;
}

double PointCloudsCalculation::GHDistance_CPLEX(PointCloud pointcloud1, PointCloud pointcloud2){
    vector<vector<double>> _pointcloud1 = pointcloud1.getPoints_2();
    vector<vector<double>> _pointcloud2 = pointcloud2.getPoints_2();
    double GHDistance = myCplexGHDSolve(_pointcloud1, _pointcloud2);
    return GHDistance;
}

double PointCloudsCalculation::myCplexGHDSolve(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2){
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    IloObjective obj(env);
    IloNumVarArray vars(env);
    IloRangeArray ranges(env);
    
    double target = -1.0;
    long m = pointclouds1.size();
    long n = pointclouds2.size();
    long totalNum = m*n;
    
    // set variable type, IloNumVarArray starts from 0.
    for (int i = 0; i < totalNum; i++){
        vars.add(IloNumVar(env, 0.0, 1.0, ILOINT));
    }
    vars.add(IloNumVar(env, 0.0, INFINITY, ILOFLOAT)); // our target
    
    // declare objective
    obj.setExpr(vars[totalNum]);
    obj.setSense(IloObjective::Minimize);
    model.add(obj);
    
    // add constraint for formula 1
    for (int i = 0; i < m; i++){
        IloNumVarArray tempVars(env,n);
        IloNumArray tempCoefs(env,n);
        IloRange tempRange(env,1.0,INFINITY);
        for (int k = 0; k < n; k++){
            tempRange.setLinearCoef(vars[i*n+k], 1.0);
        }
        model.add(tempRange);
    }
    
    // add constraint for formual 2
    for (int k = 0; k < n; k++){
        IloNumVarArray tempVars(env,m);
        IloNumArray tempCoefs(env,m);
        IloRange tempRange(env,1.0,INFINITY);
        for (int i = 0; i < m; i++){
            tempRange.setLinearCoef(vars[i*n+k], 1.0);
        }
        model.add(tempRange);
    }
    
    
    // add constraint for formula 3
    double denominator = 1.0;
    for (int i = 0; i < m; i++){
        for (int k = 0; k < n; k++){
            for(int j = 0; j < m; j++){
                for (int l = 0 ; l < n; l++){
                    if (i == j && k == l){
                        continue; // reduce the same
                    }
                    if (j*n + l <= i*n + k){
                        continue; // reduce redudant
                    }
                    denominator = myCalculateDistortion(i, j, k, l, pointclouds1, pointclouds2);
                    if (denominator == 0){
                        model.add(vars[i*n+k] + vars[j*n+l] <= 2);
                    } else {
                        model.add(vars[i*n+k] + vars[j*n+l] -(1.0/denominator)*vars[totalNum] <= 1);
                    }
                }
            }
        }
    }
    
    if(!cplex.solve()){
        cout << "cplex solve failure ! " << endl;
    }
    target = cplex.getObjValue();
    
    //    cout << "the variable 1: " << cplex.getValue(vars[0]) << endl;
    //    cout << "the variable 2: " << cplex.getValue(vars[1]) << endl;
    //    cout << "the variable m*n+1: " << cplex.getValue(vars[totalNum]) << endl;
    
    //write the .mps file
    //    cplex.exportModel("/Users/lizhe/Desktop/pointclouddataset/lalala.mps");
    
    env.end();
    return target;
}

double PointCloudsCalculation::myCalculateDistortion(const int i, const int j, const int k, const int l, const vector<vector<double>> &pointclouds1,
                           const vector<vector<double>> &pointclouds2){
    
    double diffX1 = fabs(pointclouds1[i][0] - pointclouds1[j][0]);
    double diffY1 = fabs(pointclouds1[i][1] - pointclouds1[j][1]);
    double diffZ1 = fabs(pointclouds1[i][2] - pointclouds1[j][2]);
    double diffX2 = fabs(pointclouds2[k][0] - pointclouds2[l][0]);
    double diffY2 = fabs(pointclouds2[k][1] - pointclouds2[l][1]);
    double diffZ2 = fabs(pointclouds2[k][2] - pointclouds2[l][2]);
    
    double distance1 = sqrt(diffX1 * diffX1 + diffY1 * diffY1 + diffZ1 * diffZ1);
    double distance2 = sqrt(diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2);
    
    double result = fabs(distance1 - distance2);
    //cout << "distance: " << result << " ------- distance1 " << distance1 << " --------  distance2 " << distance2 <<endl;
    return result;
}
