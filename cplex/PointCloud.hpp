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
#include <algorithm>
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
    clusterResult runKcenters(int k);
    clusterResult runKcenters2(int k);
    clusterResult runKcenters3(int k);
    
    double calculateDiameter();
    
    static double calculateClusterMaxRadius(clustersMap clusters);
    static vector<double> getClustersRadius(clustersMap clusters);
    static double getClusterRadius(Point center, vector<Point> cluster);
    
    void generateGnuplotFiles(clustersMap clusters, string address);
    void generateClustersSizeFiles(clustersMap clusters, string address);
    void generateMaxRadiusInfo(clustersMap clusters, string address);
    void generateRadiusChange(int k, string address);
    
    void calculateTheMaxMinAvgPointDistance();
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

PointCloud::clusterResult PointCloud::runKcenters(int k){
    
    vector<Point> copy_pointcloud = this->pointcloud;
    long num = this->pointcloud.size();
    vector<Point> kcenters;
    clustersMap clusters;
    
    // choose the first center randomly
    srand((unsigned)time(NULL));
    int index = rand() % num;
    copy_pointcloud[index].isCenter = true;
    kcenters.push_back(copy_pointcloud[index]);
    
    // chose the farthest point to all centers once a time
    double maxMinDistance;
    double minDistance;
    double distance;
    bool firstassign = true;
    // loop for k-1 times for the remainnning k-1 centers
    for (int count = 1; count < k; count++){
        // for all points
        maxMinDistance = 0;
        for (int i = 0; i < num; i++){
            // if it is not a center
            if(copy_pointcloud[i].isCenter){
                continue;
            }
            firstassign = true;
            // for all centers
            for(int j = 0; j < kcenters.size(); j++){
                distance = copy_pointcloud[i].distanceTo(kcenters[j]);
                if(firstassign){
                    minDistance = distance;
                    firstassign = false;
                }
                if(distance < minDistance){
                    minDistance = distance;
                }
            }
            if (minDistance > maxMinDistance){
                maxMinDistance = minDistance;
                index = i;
            }
        }
        copy_pointcloud[index].isCenter = true;
        kcenters.push_back(copy_pointcloud[index]);
    }
    
    
    // assign the remainning points to its near centers
    bool firstflag = true;
    for (int i = 0; i < copy_pointcloud.size(); i++){
        minDistance = 0;
        firstflag = true;
        for (int j = 0; j < kcenters.size(); j++){
            distance = kcenters[j].distanceTo(copy_pointcloud[i]);
            if(firstflag){
                minDistance = distance;
                index = j;
                firstflag = false;
            }
            if(distance < minDistance){
                minDistance = distance;
                index = j;
            }
        }
        clusters[kcenters[index]].push_back(copy_pointcloud[i]);
    }
    
    clusterResult result;
    result.clusters = clusters;
    result.centers = kcenters;
    return result;
}

bool wayToSort(pair<int, double> p1, pair<int, double> p2){
    return p1.second > p2.second; // sort accounding to descending order
}

PointCloud::clusterResult PointCloud::runKcenters2(int k) {
    
    vector<Point> copy_pointcloud = this->pointcloud;
    long num = this->pointcloud.size();
    vector<Point> kcenters;
    clustersMap clusters;
    vector<vector<pair<int,double>>> distanceMatrix;
    vector<pair<int,double>> subDistanceMatrix;
    int diameterIndex1 = 0, diameterIndex2 = 1;
    double distance;
    double diameter = 0;
    
    // first, calculate the vector to store the distance of each pair of points
    for (int i1 = 0; i1 < copy_pointcloud.size(); i1++){
        subDistanceMatrix.clear();
        for (int i2 = 0; i2 < copy_pointcloud.size(); i2++){
            distance = copy_pointcloud[i1].distanceTo(copy_pointcloud[i2]);
            subDistanceMatrix.push_back(pair<int, double>(copy_pointcloud[i2].index, distance));
            if (distance > diameter){
                diameter = distance;
                diameterIndex1 = i1;
                diameterIndex2 = i2;
            }
        }
        distanceMatrix.push_back(subDistanceMatrix);
    }
    
    // add the diameter point as the initial 2 centers
    copy_pointcloud[diameterIndex1].isCenter = true;
    kcenters.push_back(copy_pointcloud[diameterIndex1]);
    copy_pointcloud[diameterIndex2].isCenter = true;
    kcenters.push_back(copy_pointcloud[diameterIndex2]);
    
    // sort the distance matrix by descending order
//    for (int i = 0; i < distanceMatrix.size(); i++){
//        sort(distanceMatrix[i].begin(), distanceMatrix[i].end(), wayToSort);
//    }
    
    // loop k-2 times to find the remainnning centers
    double maxMinDistance;
    double minDistance;
    bool firstassign = true;
    int index = 0;
    // loop for k-2 times for the remainnning k-1 centers
    for (int count = 2; count < k; count++){
        // for all points
        maxMinDistance = 0;
        for (int i = 0; i < num; i++){
            // if it is not a center
            if(copy_pointcloud[i].isCenter){
                continue;
            }
            firstassign = true;
            // for all centers
            for(int j = 0; j < kcenters.size(); j++){
                distance = distanceMatrix[i][kcenters[j].index].second;
//                distance = copy_pointcloud[i].distanceTo(kcenters[j]);
                if(firstassign){
                    minDistance = distance;
                    firstassign = false;
                }
                if(distance < minDistance){
                    minDistance = distance;
                }
            }
            if (minDistance > maxMinDistance){
                maxMinDistance = minDistance;
                index = i;
            }
        }
        copy_pointcloud[index].isCenter = true;
        kcenters.push_back(copy_pointcloud[index]);
    }
    
    // assign each points to its nearest center
    bool firstflag = true;
    minDistance = 0;
    for (int i = 0; i < copy_pointcloud.size(); i++){
        minDistance = 0;
        firstflag = true;
        for (int j = 0; j < kcenters.size(); j++){
            distance = distanceMatrix[i][kcenters[j].index].second;
//            distance = kcenters[j].distanceTo(copy_pointcloud[i]);
            if(firstflag){
                minDistance = distance;
                index = j;
                firstflag = false;
            }
            if(distance < minDistance){
                minDistance = distance;
                index = j;
            }
        }
        clusters[kcenters[index]].push_back(copy_pointcloud[i]);
    }
    
    clusterResult result;
    result.clusters = clusters;
    result.centers = kcenters;
    return result;
}

PointCloud::clusterResult PointCloud::runKcenters3(int k){
    vector<Point> centers;
    clustersMap clusters;
    double sumx = 0;
    double sumy = 0;
    double sumz = 0;
    long size = 1;
    clusterResult result = this->runKcenters2(k);
    for (auto it = result.clusters.begin(); it != result.clusters.end(); it++){
        sumx = 0;
        sumy = 0;
        sumz = 0;
        size = it->second.size();
        for (int i = 0; i < size; i++){
            sumx += it->second[i].x;
            sumy += it->second[i].y;
            sumz += it->second[i].z;
        }
        Point p(sumx/size, sumy/size, sumz/size);
        clusters.insert(pair<Point, vector<Point>>(p, it->second));
        centers.push_back(p);
    }
    result.clusters = clusters;
    result.centers = centers;
    return result;
}

double PointCloud::calculateDiameter(){
    vector<vector<double>> pointclouds1 = this->getPoints_2();
    double diam1 = 0;
    double temp = 0;
    long size = pointclouds1.size();
    for (int i = 0; i < size; i++){
        for (int j = i; j < size; j++){
            temp = sqrt((pointclouds1[i][0]-pointclouds1[j][0])*(pointclouds1[i][0]-pointclouds1[j][0]) +(pointclouds1[i][1]-pointclouds1[j][1])*(pointclouds1[i][1]-pointclouds1[j][1]) + (pointclouds1[i][2]-pointclouds1[j][2])*(pointclouds1[i][2]-pointclouds1[j][2]));
            if (temp > diam1)
                diam1 = temp;
        }
    }
    return diam1;
}

double PointCloud::calculateClusterMaxRadius(clustersMap clusters){
    double maxRadius = 0;
    double maxDistance = 0;
    double distance = 0;
    Point tempCenter;
    for (auto itk = clusters.begin(); itk != clusters.end(); itk++){
        tempCenter = itk->first;
        maxDistance = 0;
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

vector<double> PointCloud::getClustersRadius(clustersMap clusters){
    
    vector<double> radius;
    
    double maxDistance = 0;
    double distance = 0;
    Point tempCenter;
    int count = 1;
    for (auto itk = clusters.begin(); itk != clusters.end(); itk++){
        tempCenter = itk->first;
        for(int i = 0; i < itk->second.size(); i++){
            distance = tempCenter.distanceTo(itk->second[i]);
            if (distance > maxDistance){
                maxDistance = distance;
            }
        }
        radius.push_back(maxDistance);
        count++;
    }
    return radius;
}

double PointCloud::getClusterRadius(Point center, vector<Point> cluster){
    double maxRadius = 0;
    double distance = 0;
    for(int i = 0; i < cluster.size(); i++){
        distance = center.distanceTo(cluster[i]);
        if(distance > maxRadius){
            maxRadius = distance;
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
    outfile << "set x label \"x\"" << endl;
    outfile << "set y label \"y\"" << endl;
    outfile << "set z label \"z\"" << endl;
    outfile << "splot ";
    for (int i = 1; i < clusterNumber; i++){
        outfile << "\"" << address+"/cluster"+to_string(i) << "\"" << " pointtype " << i << " title " << "\"" << "cluster" << i << "\",";
    }
    outfile << "\"" << address+"/cluster"+to_string(clusterNumber) << "\"" << " pointtype " << clusterNumber << " title " << "\"" << "cluster" << clusterNumber << "\"";
    outfile.close();
}

void PointCloud::generateClustersSizeFiles(clustersMap clusters, string address){
   
    mkdir(address.c_str(), 0777);
    long clusterNumber = clusters.size();
    string filename;
    filename = address+"/k="+to_string(clusterNumber);
    ofstream outfile;
    outfile.open(filename);

    double maxDistance = 0;
    double distance = 0;
    Point tempCenter;
    int count = 1;
    for (auto itk = clusters.begin(); itk != clusters.end(); itk++){
        tempCenter = itk->first;
        for(int i = 0; i < itk->second.size(); i++){
            distance = tempCenter.distanceTo(itk->second[i]);
            if (distance > maxDistance){
                maxDistance = distance;
            }
        }
        outfile << count << " " << maxDistance << endl;
        count++;
    }
    
    outfile.close();
}

void PointCloud::generateMaxRadiusInfo(clustersMap clusters, string address){
    double maxRadius = 0;
    double maxDistance = 0;
    double distance = 0;
    Point tempCenter;
    Point maxRadiusCenter;
    for (auto itk = clusters.begin(); itk != clusters.end(); itk++){
        tempCenter = itk->first;
        maxDistance = 0;
        for(int i = 0; i < itk->second.size(); i++){
            distance = tempCenter.distanceTo(itk->second[i]);
            if (distance > maxDistance){
                maxDistance = distance;
            }
        }
        if (maxDistance > maxRadius){
            maxRadius = maxDistance;
            maxRadiusCenter = itk->first;
        }
    }
    
    mkdir(address.c_str(), 0777);
    long clusterNumber = clusters.size();
    string filename;
    filename = address+"/Points-MaxRadiusCluster-k="+to_string(clusterNumber);
    ofstream outfile;
    outfile.open(filename);
    vector<Point> points = clusters[maxRadiusCenter];
    
    outfile << "maxradius: " << maxRadius << endl;
    
    for(int i = 0; i < points.size(); i++){
        outfile << points[i].x << " " << points[i].y << " " << points[i].z << endl;
    }
    outfile.close();
    
    filename = address+"/Center-MaxRadiusCluster-k="+to_string(clusterNumber);
    outfile.open(filename);
    outfile << maxRadiusCenter.x << " " << maxRadiusCenter.y << " " << maxRadiusCenter.z << endl;
    outfile.close();
}

void PointCloud::generateRadiusChange(int k, string address){
    clusterResult result = this->runKcenters2(k);
    vector<vector<double>> radius;
    vector<double> subradius;
    double distance;
    double maxdistance = 0;
    double sumx = 0;
    double sumy = 0;
    double sumz = 0;
    double avgx = 0;
    double avgy = 0;
    double avgz = 0;
    double size = 1;
    for(auto it = result.clusters.begin(); it != result.clusters.end(); it++){
        // first find radius
        Point temp = it->first;
        maxdistance = 0;
        subradius.clear();
        size = it->second.size();
        sumx = 0;
        sumy = 0;
        sumz = 0;
        for (int i = 0; i < size; i++){
            distance = temp.distanceTo(it->second[i]);
            if(distance > maxdistance){
                maxdistance = distance;
            }
            sumx += it->second[i].x;
            sumy += it->second[i].y;
            sumz += it->second[i].z;
        }
        subradius.push_back(maxdistance);
        
        avgx = sumx/size;
        avgy = sumy/size;
        avgz = sumz/size;
        Point tempcenter(avgx,avgy,avgz);
        maxdistance = 0;
        for (int i = 0; i < size; i++){
            distance = tempcenter.distanceTo(it->second[i]);
            if(distance > maxdistance){
                maxdistance = distance;
            }
        }
        subradius.push_back(maxdistance);
        radius.push_back(subradius);
    }
    
    mkdir(address.c_str(), 0777);
    string filename;
    filename = address+"/ClusterChange-k="+to_string(k);
    ofstream outfile;
    outfile.open(filename);
    for (int i = 0; i < k; i++){
        outfile << i+1 << " " << radius[i][0] << " " << radius[i][1] << endl;
    }
    outfile.close();
    
    string filename2 = address+"/command-ClusterChange-k="+to_string(k);
    outfile.open(filename2);
    outfile << "set style fill pattern 1 border" << endl;
    outfile << "set xlabel " << "\"" << "cluster" << "\"" << endl;
    outfile << "set ylabel " << "\"" << "radius" << "\"" << endl;
    outfile << "plot ";
    outfile << "\"" << filename << "\"" << " using 2:xtic(1) title \"kcenter\" with histogram,";
    outfile << "\"" << filename << "\"" << " using 3:xtic(1) title \"kcenter-avg\" with histogram";
    outfile.close();
}

void PointCloud::calculateTheMaxMinAvgPointDistance(){
    
    double distance = 0;
    double minDistance = 0;
    double maxDistance = 0;
    double sumDistance = 0;
    double avgDistance = 0;
    bool firsttry = true;
    for(int i = 0; i < pointcloud.size(); i++){
        for(int j = i; j < pointcloud.size(); j++){
            if(i == j){
                continue;
            }
            distance = pointcloud[i].distanceTo(pointcloud[j]);
            if(firsttry){
                minDistance = distance;
                firsttry = false;
            }
            sumDistance += distance;
            if(distance > maxDistance){
                maxDistance = distance;
            }
            if(distance < minDistance){
                minDistance = distance;
            }
        }
    }
    long i = pointcloud.size();
    avgDistance = sumDistance / ((i*i-i)/2);
    cout << "max distance: " << maxDistance << endl;
    cout << "min distance: " << minDistance << endl;
    cout << "avg distance: " << avgDistance << endl;
}
