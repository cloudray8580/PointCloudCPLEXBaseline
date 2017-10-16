//
//  main.cpp
//  cplex
//
//  Created by 李喆 on 2017/10/12.
//  Copyright © 2017年 李喆. All rights reserved.
//

#include <iostream>
#include <ilcplex/ilocplex.h>
#include <fstream>
#include <vector>
#include <set>
#include <math.h>
#include <string>
using namespace std;

double runSolution(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2);
double runSolutionThroughFile(const char filename[]);
double myCplexSolve(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2);
double calculateDistortion(const int i, const int j, const int k, const int l, const vector<vector<double>> &pointclouds1,
                         const vector<vector<double>> &pointclouds2);
vector<vector<double>> creatPointCloudArrayFromFile(string filename);
void printVector(vector<vector<double>> pointclouds);
vector<vector<double>> randomExtract(const vector<vector<double>> &pointclouds, int number);
double calculateUpperBound1(const vector<vector<double>> &pointclouds1, const vector<vector<double>> &pointclouds2);

int main(int argc, const char * argv[]) {
    
    vector<vector<double>> pointclouds1 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test1.pts"); // cube with length 1 in quadrant 1
    vector<vector<double>> pointclouds2 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test2.pts"); // cube with length 1 in quadrant 7
    vector<vector<double>> pointclouds3 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test3.pts"); // cube with length 2 in quadrant 1
    vector<vector<double>> pointclouds4 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test4.pts"); // 2D rectangle 2*4
    vector<vector<double>> pointclouds5 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test5.pts"); // 2D rectangle 2*2
    vector<vector<double>> pointclouds6 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test6.pts"); // 3D Up Centrum 5 points
    vector<vector<double>> pointclouds7 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test7.pts"); // 3D Down Centrum 5 points
    vector<vector<double>> pointclouds8 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test8.pts"); // 2D square with length 1
    vector<vector<double>> pointclouds9 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test9.pts"); // 2D square with length 2
    vector<vector<double>> pointclouds10 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test10.pts"); // line with length 4
    vector<vector<double>> pointclouds11 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/test11.pts"); // line with length 2
    vector<vector<double>> pointcloudsX1 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/000020.pts"); // totally 2639 points
    vector<vector<double>> pointcloudsX2 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/000907.pts"); // totally 2518 points
    
//    vector<vector<double>> _10ofX1 = randomExtract(pointcloudsX1, 10);
//    vector<vector<double>> _10ofX2 = randomExtract(pointcloudsX2, 10);
//
//    vector<vector<double>> _20ofX1 = randomExtract(pointcloudsX1, 20);
//    vector<vector<double>> _20ofX2 = randomExtract(pointcloudsX2, 20);
//
//    vector<vector<double>> _50ofX1 = randomExtract(pointcloudsX1, 50);
//    vector<vector<double>> _50ofX2 = randomExtract(pointcloudsX2, 50);
//
//    vector<vector<double>> _100ofX1 = randomExtract(pointcloudsX1, 100);
//    vector<vector<double>> _100ofX2 = randomExtract(pointcloudsX2, 100);
//
//    vector<vector<double>> _200ofX1 = randomExtract(pointcloudsX1, 200);
//    vector<vector<double>> _200ofX2 = randomExtract(pointcloudsX2, 200);
//
//    vector<vector<double>> _500ofX1 = randomExtract(pointcloudsX1, 500);
//    vector<vector<double>> _500ofX2 = randomExtract(pointcloudsX2, 500);
//
//    vector<vector<double>> _1000ofX1 = randomExtract(pointcloudsX1, 1000);
//    vector<vector<double>> _1000ofX2 = randomExtract(pointcloudsX2, 1000);
    
//    runSolution(_20ofX1, _20ofX2);
//    runSolution(pointclouds1, pointclouds3);
    
    calculateUpperBound1(pointclouds4, pointclouds5);
}

double runSolution(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2){
    time_t start,stop;
    start = time(NULL);
    double target = myCplexSolve(pointclouds1, pointclouds2); // call to my CPLEX related function
    stop = time(NULL);
    
    cout << "==========================================" << endl;
    cout << "  our target : " << target << "   ----   time usage : "<< stop-start << endl;
    cout << "==========================================" << endl;
    
    return target;
}

double runSolutionThroughFile(const char filename[]){
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    IloObjective obj(env);
    IloNumVarArray x(env);
    IloRangeArray rngs(env);
    
    // the lp_solve generated .lp file do not work
    // use the .mps file instead should be OK
    cplex.importModel(model, filename, obj, x, rngs);
    
    time_t start,stop;
    start = time(NULL);
    cplex.solve();
    stop = time(NULL);
    
    double target = cplex.getObjValue();
    
    cout << "==========================================" << endl;
    cout << "  our target : " << target << "   ----   time usage : "<< stop-start << endl;
    cout << "==========================================" << endl;
    
    env.end();
    return target;
}

double myCplexSolve(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2){
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    IloObjective obj(env);
    IloNumVarArray vars(env);
    IloRangeArray ranges(env);
    
    double target = -1.0;
    int m = pointclouds1.size();
    int n = pointclouds2.size();
    int totalNum = m*n;
    
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
    
    //write the .mps file
//    cplex.exportModel("/Users/lizhe/Desktop/pointclouddataset/lalala.mps");
    
    env.end();
    return target;
}

double calculateDistortion(const int i, const int j, const int k, const int l, const vector<vector<double>> &pointclouds1,
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

vector<vector<double>> creatPointCloudArrayFromFile(string filename){
    
    vector<vector<double>> pointclouds;
    
    ifstream infile;
    infile.open(filename);
    if(!infile)
    {
        cout << "fail to open file " << filename << endl;
    }
    
    string str;
    std::string::size_type pos1, pos2;
    const string space = " ";
    
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
        
        pointclouds.push_back(temp);
    }
    infile.close();
    return pointclouds;
}

void printVector(vector<vector<double>> pointclouds){
    long count = pointclouds.size();
    for (int i = 0; i < count ; i++)
    {
        cout << pointclouds[i][0] << " " << pointclouds[i][1] << " " << pointclouds[i][2] << endl;
    }
}

vector<vector<double>> randomExtract(const vector<vector<double>> &pointclouds, int number){
    vector<vector<double>> subpointcloud;
    set<int> indexrecord;
    int num = pointclouds.size();
    int count = number;
    int index = 0;
    srand((unsigned)time(NULL));
    while(count > 0){
        index = rand() % num; // create random variable in [0, num)
        if (indexrecord.find(index) == indexrecord.end()){ // do not repeat
            subpointcloud.push_back(pointclouds[index]);
            indexrecord.insert(index);
            count--;
        } else {
            continue;
        }
    }
    return subpointcloud;
}

// using the 1/2max(diam(x), diam(y))
double calculateUpperBound1(const vector<vector<double>> &pointclouds1, const vector<vector<double>> &pointclouds2){
    double upperbound = 0.0;
    int size1 = pointclouds1.size();
    int size2 = pointclouds2.size();
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

// do not modify the point cloud!
//vector<vector<double>> randomExtract2(const vector<vector<double>> pointclouds, int number){
//    vector<vector<double>> subpointcloud;
//    int num = pointclouds.size();
//    int count = num;
//    int index = 0;
//    srand((unsigned)time(NULL));
//    while(count > 0){
//        index = rand() % count; // create random variable in [0, num)
//        subpointcloud.push_back(pointclouds[index]);
//        pointclouds.
//    }
//    // 需要处理重复的问题！！ 用set contains？
//    return subpointcloud;
//}

