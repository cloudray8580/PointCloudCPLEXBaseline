//
//  PointCloud.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/1.
//  Copyright © 2017年 李喆. All rights reserved.
//
#pragma once
#ifndef PointCloud_hpp
#define PointCloud_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <string>
#include <sys/stat.h>
#include "Point.hpp"

#endif /* PointCloud_hpp */
using namespace std;

class PointCloud{
public:
    
    PointCloud(vector<Point> pointcloud);
    PointCloud(vector<vector<double>> pointcloud);
    PointCloud(string filename);
    
//    struct compareByPosition {
//        bool operator()(const Point a, const Point b)const{
//            if (a.x != b.x){
//                return a.x < b.x;
//            }
//            if (a.y != b.y){
//                return a.y < b.y;
//            }
//            if (a.z != b.z){
//                return a.z < b.z;
//            }
//            return false;
//        }
//    };
//
//    typedef map<Point, vector<Point>, compareByPosition> clustersMap;
    
    // I define the bool operator < in Point
    typedef map<Point, vector<Point>> clustersMap;
    
    struct clusterResult{
        clustersMap clusters;
        vector<Point> centers;
    };
    
    vector<Point> pointcloud;
    
    vector<Point> getPoints_1();
    vector<vector<double>> getPoints_2();
    
    vector<Point> randomExtractWithRepetition(int number);
    vector<Point> randomExtractWithoutRepetition(int number);
    
    vector<Point> randomExtractCenterWithMarks(int number, vector<Point> &pointcloud);
    
    clusterResult runKmedoids(int k);
    clusterResult runKmeans(int k);
    
    double calculateClusterMaxRadius(clustersMap clusters);
    
    void generateGnuplotFiles(clustersMap clusters, string address);
};

static bool operator == (const PointCloud::clustersMap map1, const PointCloud::clustersMap map2){
    for (auto it1 = map1.begin(), it2 = map2.begin(); it1 != map1.end(); it1++, it2++){
        if (it1->first != it2->first || it1->second != it2->second){
            return false;
        }
    }
    return true;
}

PointCloud::PointCloud(vector<Point> pointcloud){
    this->pointcloud = pointcloud;
}

PointCloud::PointCloud(vector<vector<double>> pointcloud){
    this->pointcloud.clear();
    for (int i = 0; i < pointcloud.size(); i++){
        this->pointcloud.push_back(Point(pointcloud[i][0],pointcloud[i][1],pointcloud[i][2],i));
    }
}

PointCloud::PointCloud(string filename){
    vector<Point> pointcloud;
    
    ifstream infile;
    infile.open(filename);
    if(!infile)
    {
        cout << "fail to open file " << filename << endl;
    }
    
    string str;
    std::string::size_type pos1, pos2;
    const string space = " ";
    
    int count = 0;
    this->pointcloud.clear();
    while(getline(infile,str))   //按行读取,遇到换行符结束
    {
        //cout<<str<<endl;
        
        vector<double> temp;
        
        pos2 = str.find(space);
        pos1 = 0;
        while(std::string::npos != pos2)
        {
            temp.push_back(atof((str.substr(pos1, pos2-pos1)).c_str()));
            
            pos1 = pos2 + space.size();
            pos2 = str.find(space, pos1);
        }
        if(pos1 != str.length())
            temp.push_back(atof((str.substr(pos1)).c_str()));
        
        Point point(temp[0],temp[1],temp[2], count);
        pointcloud.push_back(point);
        count++;
    }
    infile.close();
    this->pointcloud = pointcloud;
}

vector<Point> PointCloud::getPoints_1(){
    return this->pointcloud;
}

vector<vector<double>> PointCloud::getPoints_2(){
    vector<vector<double>> _pointcloud;
    
    for(int i = 0; i < this->pointcloud.size(); i++){
        vector<double> temp;
        temp.push_back(this->pointcloud[i].x);
        temp.push_back(this->pointcloud[i].y);
        temp.push_back(this->pointcloud[i].z);
        _pointcloud.push_back(temp);
    }
    return _pointcloud;
}

vector<Point> PointCloud::randomExtractWithRepetition(int number){
    vector<Point> result;
    long num = this->pointcloud.size();
    int index;
    srand((unsigned)time(NULL));
    for (int i = 0; i < number ; i++){
        index = rand() % num; // create random variable in [0, num)
        result.push_back(this->pointcloud[index]);
    }
    return result;
}

vector<Point> PointCloud::randomExtractWithoutRepetition(int number){
    vector<Point> result;
    vector<Point> temp = this->pointcloud;
    long num = this->pointcloud.size();
    int index;
    srand((unsigned)time(NULL));
    while(number > 0){
        index = rand() % num; // create random variable in [0, num)
        if (temp[index].isAvailable){
            result.push_back(this->pointcloud[index]);
            temp[index].isAvailable = false;
        } else {
            continue;
        }
    }
    return result;
}

vector<Point> PointCloud::randomExtractCenterWithMarks(int number, vector<Point> &pointcloud){
    vector<Point> result;
    long num = this->pointcloud.size();
    int index;
    srand((unsigned)time(NULL));
    while(number > 0){
        index = rand() % num; // create random variable in [0, num)
        if (!pointcloud[index].isCenter){
            pointcloud[index].isCenter = true;
            result.push_back(pointcloud[index]);
            number--;
        } else {
            continue;
        }
    }
    return result;
}

PointCloud::clusterResult PointCloud::runKmedoids(int k){
    
    if (k > this->pointcloud.size()){
        // error
    }
    
    // get initial centers
    vector<Point> copy_pointcloud = this->pointcloud;
    vector<Point> kcenters = randomExtractCenterWithMarks(k, copy_pointcloud);
    
    // assigne initial clusters
    clustersMap clusters;
    double distance = 0;
    double minDistance = 0;
    double firstloop = true;
    int minK = 0;
    double totalcost = 0;
    for (int i = 0; i < copy_pointcloud.size(); i++){
        firstloop = true;
        for (int j = 0; j < kcenters.size(); j++){
            if (!copy_pointcloud[i].isCenter){
                distance = copy_pointcloud[i].distanceTo(kcenters[j]);
                if (firstloop) {
                    minDistance = distance;
                    minK = j;
                    firstloop = false;
                }
                if (distance < minDistance){
                    minDistance = distance;
                    minK = j;
                }
            } else {
                continue;
            }
        }
        clusters[kcenters[minK]].push_back(copy_pointcloud[i]);
        totalcost += minDistance;
    }
    
    // start replacement loop
    bool stillChange = false;
    Point originalCenter;
    clustersMap tempClusters;
    double tempTotalcost = 0;
    do {
         cout << "new loop !!!" << endl;
        stillChange = false;
        
        // for each kcenter, try to replace it with an non-center point
        for (int i = 0; i < kcenters.size(); i++){
            for (int j = 0; j < copy_pointcloud.size(); j++){
                
                // check if point j is a center point
                if (copy_pointcloud[j].isCenter){
                    continue;
                }
                
                // replace center i with point j
                
                // 1. do reolacement in kcenter
                // store the original center, restore it when totalcost do not reduce
                originalCenter = kcenters[i];
                kcenters[i] = copy_pointcloud[j];
                kcenters[i].isCenter = true;
                
                // 2. do marks in copy_pointcloud
                copy_pointcloud[j].isCenter = true;
                copy_pointcloud[originalCenter.index].isCenter = false;
                
                // do cluster for these kcenters and calculate totalcost
                tempClusters.clear();
                tempTotalcost = 0;
                for (int m = 0; m < copy_pointcloud.size(); m++){
                    firstloop = true;
                    for (int n = 0; n < kcenters.size(); n++){
                        if (!copy_pointcloud[m].isCenter){
                            distance = copy_pointcloud[m].distanceTo(kcenters[n]);
                            if (firstloop) {
                                minDistance = distance;
                                minK = n;
                                firstloop = false;
                            }
                            if (distance < minDistance){
                                minDistance = distance;
                                minK = n;
                            }
                        } else {
                            continue;
                        }
                    }
                    tempClusters[kcenters[minK]].push_back(copy_pointcloud[m]);
                    tempTotalcost += minDistance;
                }
                
                // compare totalcost
                // if totalcost reduce, keep change, else do restortation
                if (tempTotalcost < totalcost){
                    clusters = tempClusters;
                    totalcost = tempTotalcost;
                    stillChange = true;
                    cout << "totalcost: " << totalcost << endl;
                } else {
                    kcenters[i] = originalCenter;
                    copy_pointcloud[j].isCenter = false;
                    copy_pointcloud[originalCenter.index].isCenter = true;
                }
                
            } // end of copy_pointcloud
        } // end of kcenters
    } while(stillChange);
    
    clusterResult result;
    result.clusters = clusters;
    result.centers = kcenters;
    return result;
}

PointCloud::clusterResult PointCloud::runKmeans(int k){
    if (k > this->pointcloud.size()){
        // error
    }
    
    // get initial centers
    vector<Point> copy_pointcloud = this->pointcloud;
    vector<Point> kcenters = randomExtractCenterWithMarks(k, copy_pointcloud);
    
    // start kmeans algorithm
    clustersMap clusters;
    clustersMap previous_clusters;
    bool stillChange;
    do {
        stillChange = false;
        previous_clusters = clusters;
        
        double distance = 0;
        double minDistance = 0;
        double firstloop = true;
        int minK = 0;
        
        // for each point, assign it to the nearest center
        for (int i = 0; i < copy_pointcloud.size(); i++){
            firstloop = true;
            for (int j = 0; j < kcenters.size(); j++){
                distance = copy_pointcloud[i].distanceTo(kcenters[j]);
                if (firstloop) {
                    minDistance = distance;
                    minK = j;
                    firstloop = false;
                }
                if (distance < minDistance){
                    minDistance = distance;
                    minK = j;
                }
            }
            clusters[kcenters[minK]].push_back(copy_pointcloud[i]);
        }
        
        // calculate the new average center
        long clusterSize = 0;
        double sumx = 0;
        double sumy = 0;
        double sumz = 0;
        double avgx = 0;
        double avgy = 0;
        double avgz = 0;
        int indexCount = 0;
        for (auto it = clusters.begin(); it != clusters.end(); it++){
            clusterSize = it->second.size();
            if (clusterSize == 0){
                continue;
            }
            sumx = 0;
            sumy = 0;
            sumz = 0;
            for (int i = 0; i < clusterSize; i++){
                sumx += it->second[i].x;
                sumy += it->second[i].y;
                sumz += it->second[i].z;
            }
            avgx = sumx / clusterSize;
            avgy = sumy / clusterSize;
            avgz = sumz / clusterSize;
            
            kcenters[indexCount++] = Point(avgx, avgy, avgz);
        }
        
        //compare whether change or not
        if (previous_clusters != clusters){
            stillChange = true;
        }
    } while (stillChange);
    
    clusterResult result;
    result.clusters = clusters;
    result.centers = kcenters;
    return result;
}

double PointCloud::calculateClusterMaxRadius(clustersMap clusters){
    double maxRadius = 0;
    double maxDistance = 0;
    double distance = 0;
    Point tempCenter;
    for (auto itk = clusters.begin(); itk != clusters.end(); itk++){
        tempCenter = itk->first;
        for(int i = 0; i < itk->second.size(); i++){
            distance = tempCenter.distanceTo(itk->second[i]);
            if (distance > maxDistance){
                maxDistance = distance;
            }
        }
        if (maxDistance > maxRadius){
            maxRadius = maxDistance;
        }
    }
    return maxRadius;
}

void PointCloud::generateGnuplotFiles(clustersMap clusters, string address){
    
    mkdir(address.c_str(), 0777);
    
    long clusterNumber = clusters.size();
    string filename;
    
    // create file for each cluster
    auto it = clusters.begin();
    for (int filecount = 1; filecount <= clusterNumber; filecount++, it++){
        ofstream outfile;
        filename = address+"/cluster"+to_string(filecount);
        outfile.open(filename);
        
        for (int i = 0; i < it->second.size(); i++){
            outfile << it->second[i].x << " " << it->second[i].y << " " << it->second[i].z << endl;
        }
        
        outfile.close();
    }
    
    // create the command file
    ofstream outfile;
    filename = address+"/command";
    outfile.open(filename);
    
    outfile << "splot ";
    for (int i = 1; i < clusterNumber; i++){
        outfile << "\"" << address+"/cluster"+to_string(i) << "\"" << " pointtype " << i << " title " << "\"" << " cluster" << i << "\",";
    }
    outfile << "\"" << address+"/cluster"+to_string(clusterNumber) << "\"" << " pointtype " << clusterNumber << " title " << "\"" << " cluster" << clusterNumber << "\"";
    outfile.close();
}
