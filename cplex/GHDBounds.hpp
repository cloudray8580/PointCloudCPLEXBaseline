//
//  GHDBounds.hpp
//  cplex
//
//  Created by 李喆 on 2017/10/16.
//  Copyright © 2017年 李喆. All rights reserved.
//

#ifndef GHDBounds_hpp
#define GHDBounds_hpp

#include <stdio.h>
#include <vector>
#include <set>
#include <map>
#include <iostream>
#include <math.h>
#include <ilcplex/ilocplex.h>
#endif /* GHDBounds_hpp */

using namespace std;

class Bounds{
    public:
    
    struct clusterResult{
        vector<vector<double>> kcenters;
        map<int, vector<int>> k_index_map;
        map<int, double> maxdistance; // do not use this when using k-medoids
    };
    
    struct bounds{
        double lowerbound;
        double upperbound;
    };
    
    bounds calculateBounds_Kmeans(vector<vector<double>> pointcloud1, vector<vector<double>> pointcloud2, int k);
  
    clusterResult cluster_Kmeans(int k, vector<vector<double>> pointcloud);
    
    clusterResult cluster_Kmedoids(int k, vector<vector<double>> pointcloud);
    
    double myCplexSolve(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2);
    
    double calculateDistortion(const int i, const int j, const int k, const int l, const vector<vector<double>> &pointclouds1,
                               const vector<vector<double>> &pointclouds2);
    
    double findMaxRadius(map<int, vector<int>> k_index_map, vector<vector<double>> kcenters, vector<vector<double>> pointcloud);
    
//    double findMaxRadius(map<int, vector<int>> k_index_map, vector<vector<double>> pointcloud); // not the radius in properties! is the max distance from center to another point in cluster
    
};



Bounds::bounds Bounds::calculateBounds_Kmeans(vector<vector<double>> pointcloud1, vector<vector<double>> pointcloud2, int k){
    
    time_t start,stop;
    start = time(NULL);
    
    double lowerbound = 0;
    double upperbound = 0;
    
//    clusterResult result1 = cluster_Kmeans(k, pointcloud1);
//    clusterResult result2 = cluster_Kmeans(k, pointcloud2);
    clusterResult result1 = cluster_Kmedoids(k, pointcloud1); // 返回的 k_index_map 个数不对！！！！！！可能是有些center 真的离其他点太远了？    index那里出现了超过k得数？？？？？？
    clusterResult result2 = cluster_Kmedoids(k, pointcloud2);
    
    double centerGHD = myCplexSolve(result1.kcenters, result2.kcenters) / 2;
    double center_1_GHD_estimation = findMaxRadius(result1.k_index_map, result1.kcenters, pointcloud1) * 2;
    double center_2_GHD_estimation = findMaxRadius(result2.k_index_map, result2.kcenters, pointcloud2) * 2;
    
    lowerbound = centerGHD - center_1_GHD_estimation - center_2_GHD_estimation;
    upperbound = centerGHD + center_1_GHD_estimation + center_2_GHD_estimation;
    
    bounds result;
    result.lowerbound = lowerbound;
    result.upperbound = upperbound;
    
    stop = time(NULL);
    
    cout << "======================== using kmeans ======================" << endl;
    cout << "lowerbound: " << lowerbound <<"  ---- upperbound: " << upperbound << endl;
    cout << "time usage: " << stop-start << endl;
    cout << "estimate1: " << center_1_GHD_estimation << " ---- estimation2: " << center_2_GHD_estimation << endl;
    cout << "======================== using kmeans ======================" << endl;
    
//    for (auto it = result1.k_index_map.begin(); it != result1.k_index_map.end(); it++){
//        cout << it->first << " : " << endl;
//        for(auto itv = it->second.begin(); itv != it->second.end(); itv++){
//            cout << *itv <<" ";
//        }
//        cout << endl;
//    }
    
    return result;
};

Bounds::clusterResult Bounds::cluster_Kmeans(int k, vector<vector<double>> pointcloud){
    
    vector<vector<double>> kcenters;
    map<int, double> maxdistance;
    
    long size = pointcloud.size();
    
    // find the initial K points
    srand((unsigned)time(NULL));
    set<int> indexrecord;
    int index;
    for (int i = 0; i < k; i++){
        while(true){
            index = rand() % size; // create random variable in [0, num)
            if (indexrecord.find(index) == indexrecord.end()){ // not in record
                indexrecord.insert(index);
                kcenters.push_back(pointcloud[index]);
                break;
            } else {
                continue;
            }
        }
    }
    
    // start Kmeans algorithm.
    map<int, vector<int>> k_index_map; // a mapping from k to the pointcloud index that belong to k
    map<int, vector<int>> k_index_map_first;
    double tempdistance = 0;
    bool stillChanging;
    double min = 0;
    int min_k;
    bool min_firsttry = true;
    
    double new_mean_x = 0;
    double new_mean_y = 0;
    double new_mean_z = 0;
    double sum_x = 0;
    double sum_y = 0;
    double sum_z = 0;
    
    int tempindex;
    
    do{
        stillChanging = false;
        // for each element, find its nearest k
        for(int i = 0; i < size; i++){
            min_firsttry = true;
            for (int j = 0; j < k; j++){
                tempdistance = sqrt((pointcloud[i][0] - kcenters[j][0])*(pointcloud[i][0] - kcenters[j][0]) + (pointcloud[i][1] - kcenters[j][1])*(pointcloud[i][1] - kcenters[j][1]) + (pointcloud[i][2] - kcenters[j][2])*(pointcloud[i][2] - kcenters[j][2]));
                if(min_firsttry){
                    min = tempdistance;
                    min_k = j;
                    min_firsttry = false;
                }
                if (tempdistance > maxdistance[j]){ // will radius initially be 0 ??????????????/
                    maxdistance[j] = tempdistance;
                }
                if(tempdistance < min){
                    min = tempdistance;
                    min_k = j;
                }
            }
            k_index_map[min_k].push_back(i);
        }
        
        
        // recalculate the kmeans center
        for (int i = 0; i < k; i++){
            long subsize = k_index_map[i].size();
            sum_x = 0;
            sum_y = 0;
            sum_z = 0;
            for (int j = 0; j < subsize; j++){
                tempindex = k_index_map[i][j];
                sum_x += pointcloud[tempindex][0];
                sum_y += pointcloud[tempindex][1];
                sum_z += pointcloud[tempindex][2];
            }
            new_mean_x = sum_x / subsize;
            new_mean_y = sum_y / subsize;
            new_mean_z = sum_z / subsize;
            
            // here I used the average instead of a closest point in the point set !!!!!!!!
            kcenters[i][0] = new_mean_x;
            kcenters[i][1] = new_mean_y;
            kcenters[i][2] = new_mean_z;
        }
        
        // judge if need to loop again
        if (k_index_map_first != k_index_map){ // compare their value, is that right??????????????????? , or the first_loop
            stillChanging = true;
            k_index_map_first = k_index_map;
            k_index_map.clear();
        } else {
            stillChanging = false;;
        }
        
    } while(stillChanging);
    
    clusterResult result;
    result.kcenters = kcenters;
    result.k_index_map = k_index_map;
    result.maxdistance = maxdistance;
    
    return result;
}

Bounds::clusterResult Bounds::cluster_Kmedoids(int k, vector<vector<double>> pointcloud){
    
    vector<vector<double>> kcenters;
    map<int, double> maxdistance;
    
    long size = pointcloud.size();
    
    // find the initial K points
    srand((unsigned)time(NULL));
    set<int> indexrecord;
    int index;
    for (int i = 0; i < k; i++){
        while(true){
            index = rand() % size; // create random variable in [0, num)
            if (indexrecord.find(index) == indexrecord.end()){ // not in record
                indexrecord.insert(index);
                kcenters.push_back(pointcloud[index]);
                break;
            } else {
                continue;
            }
        }
    }
    
    // start K-medioids algorithm.
    map<int, vector<int>> k_index_map; // a mapping from k to the pointcloud index that belong to k
    map<int, vector<int>> k_index_map_first;
    double tempdistance = 0;
    bool stillChanging;
    double min = 0;
    int min_k;
    bool min_firsttry = true;
    double totalcost = 0;
    
    // for each element, find its nearest k
    for(int i = 0; i < size; i++){
        min_firsttry = true;
        for (int j = 0; j < k; j++){
            tempdistance = sqrt((pointcloud[i][0] - kcenters[j][0])*(pointcloud[i][0] - kcenters[j][0]) + (pointcloud[i][1] - kcenters[j][1])*(pointcloud[i][1] - kcenters[j][1]) + (pointcloud[i][2] - kcenters[j][2])*(pointcloud[i][2] - kcenters[j][2]));
            if(min_firsttry){
                min = tempdistance;
                min_k = j;
                min_firsttry = false;
            }
            if (tempdistance > maxdistance[j]){
                maxdistance[j] = tempdistance;
            }
            if(tempdistance < min){
                min = tempdistance;
                min_k = j;
            }
        }
        k_index_map[min_k].push_back(i);
        totalcost += min;
    }
    
    // start looping
    do{
        stillChanging = false;
        
        vector<vector<double>> tempkcenters;
        map<int, vector<int>> temp_k_index_map;
        double temptotalcost = 0;
        set<int> tempCenterRecord;
        
        // recalculate the k-modoids center
        for (int i = 0; i < k; i++){ // for each medoid
            for (int j = 0; j < pointcloud.size(); j++){
                
                // for each point not a medoid
                if (indexrecord.find(j) == indexrecord.end()){//.....？？？？这里会不会有问题. 会！因为 第一：没有改变 第二：下面重新分配的时候没有做中心点排除
                    
                    // use this point as the Kth medoid
                    tempkcenters = kcenters;
                    tempkcenters[i] = pointcloud[j];
                    temp_k_index_map.clear();
                    temptotalcost = 0;
                    tempCenterRecord = indexrecord;
                    
                    // calculate the distance from
                    for (int m = 0; m < pointcloud.size(); m++){
                        min_firsttry = true;
                        for (int n = 0; n < k; n++){
                            tempdistance = sqrt((pointcloud[m][0] - tempkcenters[n][0])*(pointcloud[m][0] - tempkcenters[n][0]) + (pointcloud[m][1] - tempkcenters[n][1])*(pointcloud[m][1] - tempkcenters[n][1]) + (pointcloud[m][2] - tempkcenters[n][2])*(pointcloud[m][2] - tempkcenters[n][2]));
                            if(min_firsttry){
                                min = tempdistance;
                                min_k = j;
                                min_firsttry = false;
                            }
                            if(tempdistance < min){
                                min = tempdistance;
                                min_k = n;
                            }
                        }
                        temp_k_index_map[min_k].push_back(m); // do distribution
                        temptotalcost += min; // calculate the cost after replacement
                    }
                } else {
                    continue;
                }
                
                // judge if it is a good change
                if (temptotalcost < totalcost){
                    totalcost = temptotalcost;
                    kcenters = tempkcenters;
                    k_index_map = temp_k_index_map;
                    stillChanging = true;
                }
            }
        }
        
        // judge if need to loop again
//        if (k_index_map_first != k_index_map){ // compare their value, is that right?
//            stillChanging = true;
//            k_index_map_first = k_index_map;
//            k_index_map.clear();
//        } else {
//            stillChanging = false;
//        }
        
    } while(stillChanging);
    
    Bounds::clusterResult result;
    result.kcenters = kcenters;
    result.k_index_map = k_index_map;
    result.maxdistance = maxdistance; //invalid
    
    return result;
}

double Bounds::findMaxRadius(map<int, vector<int>> k_index_map, vector<vector<double>> kcenters, vector<vector<double>> pointcloud){
    
    double maxRadius = 0;
    long clustersize;
    int centerindex, pointindex;
    double tempdistance = 0;
    double radius = 0;
    
    for (auto it = k_index_map.begin(); it != k_index_map.end(); it++){
        clustersize = it->second.size();
        centerindex = it->first; // 这个是kcenter的index吧!!!!
        radius = 0;
        for (int i= 0; i < clustersize; i++){
            pointindex = it->second[i];
            tempdistance = sqrt((kcenters[centerindex][0]-pointcloud[pointindex][0])*(kcenters[centerindex][0]-pointcloud[pointindex][0]) + (kcenters[centerindex][1]-pointcloud[pointindex][1])*(kcenters[centerindex][1]-pointcloud[pointindex][1]) + (kcenters[centerindex][2]-pointcloud[pointindex][2])*(kcenters[centerindex][2]-pointcloud[pointindex][2]));
            if(tempdistance > radius){
                radius = tempdistance;
            }
        }
        if (radius > maxRadius){
            maxRadius = radius;
        }
    }
    
    return maxRadius;
}

//double Bounds::findMaxRadius(map<int, vector<int>> k_index_map, vector<vector<double>> pointcloud){
//    double maxRadius = 0;
//    double radius = 0;
//    double max = 0;
//    bool firsttry = true;
//    double tempdistance = 0;
//    int index1 = 0;
//    int index2 = 0;
//    for (int i = 0; i < k_index_map.size(); i++){
//        firsttry = true;
//        for (int j = 0; j < k_index_map[i].size(); j++){
//            index1 = k_index_map[i][j];
//            max = 0;
//            for (int k = 0; k < k_index_map[i].size(); k++){
//                index2 = k_index_map[i][k];
//                tempdistance = sqrt(((pointcloud[index1][0] - pointcloud[index2][0])*(pointcloud[index1][0] - pointcloud[index2][0])) + ((pointcloud[index1][1] - pointcloud[index2][1])*(pointcloud[index1][1] - pointcloud[index2][1])) + ((pointcloud[index1][2] - pointcloud[index2][2])*(pointcloud[index1][2] - pointcloud[index2][2])));
//                if (tempdistance > max){
//                    max = tempdistance;
//                }
//            }
//            if (firsttry){
//                radius = max;
//                firsttry = false;
//            }
//            if (max < radius){
//                radius = max;
//            }
////            cout << "max: " << max << endl;
//        }
////        cout << "radius: " << radius << endl;
////        cout << "===========" << endl;
//        if (radius > maxRadius){
//            maxRadius = radius;
//        }
//    }
////    cout << "maxRadius: " << maxRadius << endl;
//    return maxRadius;
//}

double Bounds::myCplexSolve(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2){
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
                    denominator = calculateDistortion(i, j, k, l, pointclouds1, pointclouds2);
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

double Bounds::calculateDistortion(const int i, const int j, const int k, const int l, const vector<vector<double>> &pointclouds1,
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
