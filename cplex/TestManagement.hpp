//
//  TestManagement.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/2.
//  Copyright © 2017年 李喆. All rights reserved.
//

#ifndef TestManagement_hpp
#define TestManagement_hpp

#include <stdio.h>
#include "PointCloud.hpp"
#include "ConfigHelper.hpp"
#include "PointCloudsCalculation.hpp"
#include <vector>
#include <map>
#include <cstdlib>
using namespace std;

#endif /* TestManagement_hpp */

class TestManagement{
public:
    
    TestManagement(string configFilePath);
    
    map<string, string> configuration;
    
    struct testResult{
        int k;
        time_t running_time;
        double upperbound;
        double lowerbound;
        double maxradius1;
        double maxradius2;
    };
    
    struct testResultOneDatasetMultiMethod{
        int pointcloudsize1;
        int pointcloudsize2;
        map<string, vector<testResult>> results;
    };
    
    vector<vector<double>> kcenter_radius1;
    vector<vector<double>> kcenter_radius2;
    
    void run();
    void runCalculationAllFileAllMethod();
    map<string, vector<testResult>> runCalculationAllMethod(PointCloud p1, PointCloud p2, bool kcenters, bool kmeans, bool kmedoids, bool ub1, bool ub2, bool ub3, bool lb1, bool lb2, bool lb3, bool lb4);
    
    vector<testResult> runCalculationCluster(int k1, int k2, int footstep, PointCloud p1, PointCloud p2, int method);
    
    void generateGnuplotDataAndCommand_runningtime(vector<TestManagement::testResult> results, string address);
    void generateGnuplotDataAndCommand_bounds(vector<TestManagement::testResult> results, string address);
    void generateGnuplotDataAndCommand_bounds_UBLB(vector<TestManagement::testResult> results, string address);
    void generateGnuplotDataAndCommand_bounds_AllMethod(map<string, vector<testResult>> results, string address);
    
    void generateGnuplotDataAndCommand_runningtime_multiFile(vector<testResultOneDatasetMultiMethod>, string address);
    void generateGnuplotDataAndCommand_bounds_multiFile(vector<testResultOneDatasetMultiMethod>, string address);
    
    void generateGnuplotDataAndCommand_clustersSize(vector<vector<double>> clusterSize, string address, string suffix);
    
    vector<PointCloud> loadDataFromDirectory(string folderPath);
    vector<PointCloud> loadData(int type);
};

TestManagement::TestManagement(string configFilePath){
    ifstream infile;
    infile.open(configFilePath);
    if(!infile)
    {
        cout << "fail to open configuration file " << configFilePath << endl;
    }
    map<string, string> configuration;
    ReadConfig(configFilePath, configuration);
    this->configuration = configuration;
}

void TestManagement::run(){
    
    // get configuration of methods used
    bool kcenters = false;
    bool kmeans = false;
    bool kmedoids = false;
    bool ub1 = false;
    bool ub2 = false;
    bool ub3 = false;
    bool lb1 = false;
    bool lb2 = false;
    bool lb3 = false;
    bool lb4 = false;
    
    string plotMethod = "kcenters";
    if (this->configuration["run_kcenters"] == "true"){
        kcenters = true;
        plotMethod = "kcenters";
    }
    if (this->configuration["run_kmeans"] == "true"){
        kmeans = true;
        plotMethod = "kmeans";
    }
    if (this->configuration["run_kmedoids"] == "true"){
        kmedoids = true;
        plotMethod = "kmedoids";
    }
    if (this->configuration["run_ub1"] == "true"){
        ub1 = true;
    }
    if (this->configuration["run_ub2"] == "true"){
        ub2 = true;
    }
    if (this->configuration["run_ub3"] == "true"){
        ub3 = true;
    }
    if (this->configuration["run_lb1"] == "true"){
        lb1 = true;
    }
    if (this->configuration["run_lb2"] == "true"){
        lb2 = true;
    }
    if (this->configuration["run_lb3"] == "true"){
        lb3 = true;
    }
    if (this->configuration["run_lb4"] == "true"){
        lb4 = true;
    }
    
    // run one pair of dataset or all of them
    if (this->configuration["use_target"] == "true"){
        string address1 = this->configuration["pointcloud1"];
        string address2 = this->configuration["pointcloud2"];
        PointCloud p1(address1);
        PointCloud p2(address2);
        map<string, vector<testResult>> results = runCalculationAllMethod(p1, p2, kcenters, kmeans, kmedoids, ub1, ub2, ub3, lb1, lb2, lb3, lb4);
        string plotaddress = this->configuration["gnuplot_path"];
        generateGnuplotDataAndCommand_runningtime(results[plotMethod], plotaddress);
        generateGnuplotDataAndCommand_bounds(results[plotMethod], plotaddress);
        generateGnuplotDataAndCommand_bounds_UBLB(results[plotMethod], plotaddress);
        generateGnuplotDataAndCommand_bounds_AllMethod(results, plotaddress);
        generateGnuplotDataAndCommand_clustersSize(kcenter_radius1, plotaddress, "p1");
        generateGnuplotDataAndCommand_clustersSize(kcenter_radius2, plotaddress, "p2");
    } else {
        return;
    }
}

map<string, vector<TestManagement::testResult>> TestManagement::runCalculationAllMethod(PointCloud p1, PointCloud p2, bool kcenters, bool kmeans, bool kmedoids, bool ub1, bool ub2, bool ub3, bool lb1, bool lb2, bool lb3, bool lb4){
    
    kcenter_radius1.clear();
    kcenter_radius2.clear();
    
    map<string, vector<testResult>> results;
    if(kcenters){
        int k1 = atoi(this->configuration["k_start"].c_str());
        int k2 = atoi(this->configuration["k_end"].c_str());
        int footstep = atoi(this->configuration["k_footstep"].c_str());
        vector<testResult> results_kmeans = runCalculationCluster(k1, k2, footstep, p1, p2, 2);
        results["kcenters"] = results_kmeans;
    }
    
    if(kmeans){
        int k1 = atoi(this->configuration["k_start"].c_str());
        int k2 = atoi(this->configuration["k_end"].c_str());
        int footstep = atoi(this->configuration["k_footstep"].c_str());
        vector<testResult> results_kmeans = runCalculationCluster(k1, k2, footstep, p1, p2, 0);
        results["kmeans"] = results_kmeans;
    }
    
    if(kmedoids){
        int k1 = atoi(this->configuration["k_start"].c_str());
        int k2 = atoi(this->configuration["k_end"].c_str());
        int footstep = atoi(this->configuration["k_footstep"].c_str());
        vector<testResult> results_kmedoids = runCalculationCluster(k1, k2, footstep, p1, p2, 1);
        results["kmedoids"] = results_kmedoids;
    }
    
    if(ub1){
        PointCloudsCalculation pcc;
        double ub1 = pcc.Upperbound_UB1(p1, p2);
        testResult result;
        result.upperbound = ub1;
        vector<testResult> results_ub1;
        results_ub1.push_back(result);
        results["ub1"] = results_ub1;
    }
    
    if(ub2){
        
    }
    
    if(ub3){
        int extremeset_size = atoi(this->configuration["extremeset_size"].c_str());
        double delta = atof(this->configuration["delta"].c_str());
        int referenceset_size = atoi(this->configuration["referenceset_size"].c_str());
        double tolerance = atof(this->configuration["tolerance"].c_str());
        
        PointCloudsCalculation pcc;
        double ub3 = pcc.Upperbound_UB3(p1, p2, extremeset_size, delta, referenceset_size, tolerance);
        testResult result;
        result.upperbound = ub3;
        vector<testResult> results_ub3;
        results_ub3.push_back(result);
        results["ub3"] = results_ub3;
    }
    
    if(lb1){
        PointCloudsCalculation pcc;
        double lb1 = pcc.Lowerbound_LB1(p1, p2);
        testResult result;
        result.lowerbound = lb1;
        vector<testResult> results_lb1;
        results_lb1.push_back(result);
        results["lb1"] = results_lb1;
    }
    
    if(lb2){
        
    }
    
    if(lb3){
        
    }
    
    if(lb4){
        PointCloudsCalculation pcc;
        double lb4 = pcc.Lowerbound_LB4(p1, p2);
        testResult result;
        result.lowerbound = lb4;
        vector<testResult> results_lb4;
        results_lb4.push_back(result);
        results["lb4"] = results_lb4;
    }
    
    return results;
}

// method : 0 -- kmeans  1 -- kmedoids
vector<TestManagement::testResult> TestManagement::runCalculationCluster(int k1, int k2, int footstep, PointCloud p1, PointCloud p2, int method){
    
    bool flag = false;
    if (this->configuration["generate_cluster_graph"] == "true"){
        flag = true;
    }
    string plotaddress = this->configuration["gnuplot_path"];
    
    cout << "### running clustering..." << endl;
    time_t start, end;
    PointCloudsCalculation pcc;
    PointCloudsCalculation::LB_UB bounds;
    vector<testResult> results;
    for (int k = k1; k <= k2; k += footstep){
        cout << "running, k = " << k << endl;
        testResult result;
        start = time(NULL);
        if (method == 0){
            bounds = pcc.LowerboundAndUpperbound_kmeans(k, p1, p2);
        } else if(method == 1){
            bounds = pcc.LowerboundAndUpperbound_kmedoids(k, p1, p2);
        } else if(method == 2){
//            bounds = pcc.LowerboundAndUpperbound_kcenters(k, p1, p2);
//            bounds = pcc.LowerboundAndUpperbound_kcenters_upgradelb(k, p1, p2);
//            bounds = pcc.LowerboundAndUpperbound_kcenters_GlobalMatching(k, p1, p2);
            bounds = pcc.LowerboundAndUpperbound_kcenters_MyApproach3(k, p1, p2);
        }
        end = time(NULL);
        result.k = k;
        result.running_time = end-start;
        result.upperbound = bounds.upperbound;
        result.lowerbound = bounds.lowerbound;
        result.maxradius1 = bounds.maxRadius1;
        result.maxradius2 = bounds.maxRadius2;
        results.push_back(result);
        if (flag){
            p1.generateGnuplotFiles(bounds.clusters1, plotaddress+"/clusters-p1-k"+to_string(k));
            p2.generateGnuplotFiles(bounds.clusters2, plotaddress+"/clusters-p2-k"+to_string(k));
            kcenter_radius1.push_back(p1.getClustersRadius(bounds.clusters1));
            kcenter_radius2.push_back(p2.getClustersRadius(bounds.clusters1));
//            p1.generateClustersSizeFiles(bounds.clusters1, plotaddress+"/clustersSize-p1-k"+to_string(k));
//            p1.generateClustersSizeFiles(bounds.clusters1, plotaddress+"/clustersSize-p2-k"+to_string(k));
        }
    }
    return results;
}

void TestManagement::generateGnuplotDataAndCommand_runningtime(vector<TestManagement::testResult> results, string address){
    
    mkdir(address.c_str(), 0777);
    
    long resultsNumber = results.size();
    string filename;
    ofstream outfile;
    filename = address+"/k-runtime.dat";
    outfile.open(filename);
    // create file
    for (int i = 0; i < resultsNumber; i++){
        outfile << results[i].k << " " << results[i].running_time << endl;
    }
    outfile.close();
    
    // create the command file
    string filename_cmd = address+"/command-k-runtime";
    outfile.open(filename_cmd);
    
    outfile << "set xlabel " << "\"" << "k" << "\"" << endl;
    outfile << "set ylabel " << "\"" << "running time(s)" << "\"" << endl;
    outfile << "plot ";
    outfile << "\"" << filename << "\"" << " using 1:2 w lp pt 5 title \"running-time\"" << endl;
    outfile.close();
}

void TestManagement::generateGnuplotDataAndCommand_bounds(vector<TestManagement::testResult> results, string address){
    
    mkdir(address.c_str(), 0777);
    
    long resultsNumber = results.size();
    string filename;
    ofstream outfile;
    filename = address+"/k-boundsdiff.dat";
    outfile.open(filename);
    // create file
    for (int i = 0; i < resultsNumber; i++){
        outfile << results[i].k << " " << results[i].upperbound-results[i].lowerbound << endl;
    }
    outfile.close();
    
    // create the command file
    string filename_cmd = address+"/command-k-boundsdiff";
    outfile.open(filename_cmd);
    
    outfile << "set xlabel " << "\"" << "k" << "\"" << endl;
    outfile << "set ylabel " << "\"" << "Upperbound-Lowerbound" << "\"" << endl;
    outfile << "plot ";
    outfile << "\"" << filename << "\"" << " using 1:2 w lp pt 5 title \"Upperbound-Lowerbound\"" << endl;
    outfile.close();
}

void TestManagement::generateGnuplotDataAndCommand_bounds_UBLB(vector<TestManagement::testResult> results, string address){
    
    mkdir(address.c_str(), 0777);
    
    long resultsNumber = results.size();
    string filename;
    ofstream outfile;
    filename = address+"/k-bounds.dat";
    outfile.open(filename);
    // create file
    for (int i = 0; i < resultsNumber; i++){
        outfile << results[i].k << " " << results[i].upperbound << " " << results[i].lowerbound << " "<< results[i].maxradius1 << " " << results[i].maxradius2 << endl;
    }
    outfile.close();
    
    // create the command file
    string filename_cmd = address+"/command-k-bounds";
    outfile.open(filename_cmd);
    
    outfile << "set xlabel " << "\"" << "k" << "\"" << endl;
    outfile << "set ylabel " << "\"" << "distance" << "\"" << endl;
    outfile << "plot ";
    outfile << "\"" << filename << "\"" << " using 1:2 w lp pt 5 title \"Upperbound\"," << "\"" << filename << "\"" << " using 1:3 w lp pt 7 title \"Lowerbound\"," << "\"" << filename << "\"" << " using 1:4 w lp pt 9 title \"maxRadius1\"," << "\"" << filename << "\"" << " using 1:5 w lp pt 11 title \"maxRadius2\"" << endl;
    outfile.close();
}

void TestManagement::generateGnuplotDataAndCommand_bounds_AllMethod(map<string, vector<testResult>> results, string address){
    mkdir(address.c_str(), 0777);
    
    long resultsNumber = results.size();
    string filename;
    ofstream outfile;
    filename = address+"/k-bounds-all.dat";
    outfile.open(filename);
    
    vector<testResult> kcenters = results["kcenters"];
    vector<testResult> ub1 = results["ub1"];
    vector<testResult> lb1 = results["lb1"];
    // create file
    for (int i = 0; i < kcenters.size(); i++){
        outfile << kcenters[i].k << " " << kcenters[i].upperbound << " " << kcenters[i].lowerbound << " "<< ub1[0].upperbound << " " << lb1[0].lowerbound << endl;
    }
    outfile.close();
    
    // create the command file
    string filename_cmd = address+"/command-k-bounds-all";
    outfile.open(filename_cmd);
    
    outfile << "set xlabel " << "\"" << "k" << "\"" << endl;
    outfile << "set ylabel " << "\"" << "distance" << "\"" << endl;
    outfile << "plot ";
    outfile << "\"" << filename << "\"" << " using 1:2 w lp pt 5 title \"k-Upperbound\"," << "\"" << filename << "\"" << " using 1:3 w lp pt 7 title \"k-Lowerbound\"," << "\"" << filename << "\"" << " using 1:4 w lp pt 9 title \"upperbound1\"," << "\"" << filename << "\"" << " using 1:5 w lp pt 11 title \"lowerbound1\"" << endl;
    outfile.close();
}

void TestManagement::generateGnuplotDataAndCommand_clustersSize(vector<vector<double>> clusterSize, string address, string suffix){
//    mkdir(address.c_str(), 0777);
    
    long resultsNumber = clusterSize.size();
    long maxSize = clusterSize[resultsNumber-1].size();
    string filename;
    ofstream outfile;
    filename = address+"/"+suffix+"-k-clusterSize.dat";
    outfile.open(filename);
    
    for(int i = 0; i < maxSize; i++){
        outfile << i+1 << " ";
        for(int j = 0; j < resultsNumber; j++){
            if(i < clusterSize[j].size()){
                outfile << clusterSize[j][i] << " ";
            } else {
                outfile << 0 << " ";
            }
        }
        outfile << endl;
    }
    
    outfile.close();
    
    // create the command file
    string filename_cmd = address+"/"+suffix+"-command-k-clusterSize";
    outfile.open(filename_cmd);
    
    outfile << "set style fill pattern 1 border" << endl;
    outfile << "set xlabel " << "\"" << "cluster" << "\"" << endl;
    outfile << "set ylabel " << "\"" << "radius" << "\"" << endl;
    outfile << "plot ";
    for(int i = 0; i < resultsNumber-1; i++){
        outfile << "\"" << filename << "\"" << " using " << i+2 << ":xtic(1) title \"k=" << clusterSize[i].size() << "\" with histogram,";
    }
    outfile << "\"" << filename << "\"" << " using " << resultsNumber+1 << ":xtic(1) title \"k=" << clusterSize[resultsNumber-1].size() << "\" with histogram";
    outfile.close();
}
