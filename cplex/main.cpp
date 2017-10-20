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
#include <math.h>
#include <string>
#include <vector>
#include <set>
#include <map>

using namespace std;

struct extremesetAndRemainningset{
    vector<vector<double>> extremeset;
    vector<vector<double>> remainningset;
};


double runSolution(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2);

double runSolutionThroughFile(const char filename[]);

double myCplexSolve(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2);

double calculateDistortion(const int i, const int j, const int k, const int l, const vector<vector<double>> &pointclouds1,
                         const vector<vector<double>> &pointclouds2);

vector<vector<double>> creatPointCloudArrayFromFile(string filename);

void printVector(vector<vector<double>> pointclouds);

vector<vector<double>> randomExtract(const vector<vector<double>> &pointclouds, int number);

double calculateUpperBound1(const vector<vector<double>> &pointclouds1, const vector<vector<double>> &pointclouds2);

double calculateUpperBound2(const vector<vector<double>> &pointclouds1, const vector<vector<double>> &pointclouds2, int extremesetSize, double delta, int referencesetSize, double tolerance);

extremesetAndRemainningset extremeSet(const vector<vector<double>> pointclouds, int number);

multimap<vector<double>,vector<double>> calculateExtremeSetRelation(const vector<vector<double>> extremeset1, const vector<vector<double>> extremeset2, double delta);

multimap<vector<double>,vector<double>> incrementExtendCorrespondence(vector<vector<double>> extremeset1, vector<vector<double>> extremeset2, vector<vector<double>> referenceset1,  vector<vector<double>> referenceset2, multimap<vector<double>,vector<double>> relation);

multimap<vector<double>,vector<double>> extendCorrespondence(vector<vector<double>> pointcloud1, vector<vector<double>> pointcloud2, multimap<vector<double>,vector<double>> relation);

multimap<vector<double>,vector<double>> improveCorrespondence(const vector<vector<double>> pointcloud1, const vector<vector<double>> pointcloud2, multimap<vector<double>,vector<double>> relation, double tolerance);

double calculateDistortionDifference(multimap<vector<double>,vector<double>> relation1, multimap<vector<double>,vector<double>> relation2);

double calculateRelationDistortion(multimap<vector<double>,vector<double>> relation);

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
//    vector<vector<double>> _30ofX1 = randomExtract(pointcloudsX1, 30);
//    vector<vector<double>> _30ofX2 = randomExtract(pointcloudsX2, 30);
//
//    vector<vector<double>> _40ofX1 = randomExtract(pointcloudsX1, 40);
//    vector<vector<double>> _40ofX2 = randomExtract(pointcloudsX2, 40);
//
//    vector<vector<double>> _50ofX1 = randomExtract(pointcloudsX1, 50);
//    vector<vector<double>> _50ofX2 = randomExtract(pointcloudsX2, 50);

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
    
//    runSolution(_50ofX1, _50ofX2);
    
//    runSolution(pointclouds1, pointclouds3);
    
    calculateUpperBound1(pointcloudsX1, pointcloudsX2);
    
    calculateUpperBound2(pointcloudsX1, pointcloudsX2, 20, 0.5, 50, 0.00001);
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
    long m = pointclouds1.size();
    long n = pointclouds2.size();
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
    
//    cout << "the variable 1: " << cplex.getValue(vars[0]) << endl;
//    cout << "the variable 2: " << cplex.getValue(vars[1]) << endl;
//    cout << "the variable m*n+1: " << cplex.getValue(vars[totalNum]) << endl;
    
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
    long num = pointclouds.size();
    long count = number;
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

// using the extreme set way
double calculateUpperBound2(const vector<vector<double>> &pointclouds1, const vector<vector<double>> &pointclouds2, int extremesetSize, double delta, int referencesetSize, double tolerance){
    
    time_t start,stop;
    start = time(NULL);
    
    double upperbound = 0.0;
    
    int count = 1;
    
    extremesetAndRemainningset result1 = extremeSet(pointclouds1, extremesetSize);
    extremesetAndRemainningset result2 = extremeSet(pointclouds2, extremesetSize);
    
    multimap<vector<double>,vector<double>> relation = calculateExtremeSetRelation(result1.extremeset, result2.extremeset, delta);
//    for (auto it = relation.begin(); it != relation.end(); it++){
//        cout << count << " : " << it->first[0] << " " << it->first[1] << " " << it->first[2] <<" --- " << it->second[0] << " " << it->second[1] << " " << it->second[2] << endl;
//        count++;
//    }
//    cout << "=================" << endl;
    
    vector<vector<double>> referenceset1 = randomExtract(result1.remainningset, referencesetSize-extremesetSize);
    vector<vector<double>> referenceset2 = randomExtract(result2.remainningset, referencesetSize-extremesetSize);
    
//    for (int i = 0; i < referenceset1.size(); i++){
//        cout << "A-point " << i << " : " << referenceset1[i][0] << " " << referenceset1[i][1] << " " << referenceset1[i][2] << endl;
//    }
//    cout << "=================" << endl;
//    for (int i = 0; i < referenceset2.size(); i++){
//        cout << "B-point " << i << " : " << referenceset2[i][0] << " " << referenceset2[i][1] << " " << referenceset2[i][2] << endl;
//    }
//    cout << "=================" << endl;
    
    multimap<vector<double>,vector<double>> relation2 = incrementExtendCorrespondence(result1.extremeset, result2.extremeset, referenceset1, referenceset2, relation);
//    count = 1;
//    for (auto it = relation2.begin(); it != relation2.end(); it++){
//        cout << count << " : " << it->first[0] << " " << it->first[1] << " " << it->first[2] <<" --- " << it->second[0] << " " << it->second[1] << " " << it->second[2] << endl;
//        count++;
//    }
//    cout << "=================" << endl;
    
    multimap<vector<double>,vector<double>> relation3 = extendCorrespondence(pointclouds1, pointclouds2, relation2); // 最后一个关系的第二个是空的！！！！！！
//    count = 1;
//    for (auto it = relation3.begin(); it != relation3.end(); it++){
//        cout << count << " : " << it->first[0] << " " << it->first[1] << " " << it->first[2] <<" --- " << it->second[0] << " " << it->second[1] << " " << it->second[2] << endl;
//        count++;
//    }
//    cout << "=================" << endl;
    
  
    multimap<vector<double>,vector<double>> relation4 = improveCorrespondence(pointclouds1, pointclouds2, relation3, tolerance);
//    count = 1;
//    for (auto it = relation4.begin(); it != relation4.end(); it++){
//        cout << count << " : " << it->first[0] << " " << it->first[1] << " " << it->first[2] <<" --- " << it->second[0] << " " << it->second[1] << " " << it->second[2] << endl;
//        count++;
//    }
//    cout << "=================" << endl;
    
    upperbound = calculateRelationDistortion(relation4) / 2;
    
    stop = time(NULL);
    
    cout << "The upper bound for method2 : " << upperbound << "  -----  time usage : " << stop-start << endl;
    
    return upperbound;
}

extremesetAndRemainningset extremeSet(const vector<vector<double>> pointclouds, int number){
    
    vector<vector<double>> extremeset;
    vector<vector<double>> remainningset = pointclouds;
    
    set<int> extremeindex;
    
//    if (number < 2){
//        cout << "not valid extreme set size" << endl;
//        return extremeset;
//    }
    
    long size = pointclouds.size();
    int indexDiameter1 = 0;
    int indexDiameter2 = 0;
    double diam = 0;
    double temp = 0;
    for (int i = 0; i < size; i++){
        for (int j = i; j < size; j++){
            temp = sqrt((pointclouds[i][0]-pointclouds[j][0])*(pointclouds[i][0]-pointclouds[j][0]) +(pointclouds[i][1]-pointclouds[j][1])*(pointclouds[i][1]-pointclouds[j][1]) + (pointclouds[i][2]-pointclouds[j][2])*(pointclouds[i][2]-pointclouds[j][2]));
            if (temp > diam){
                diam = temp;
                indexDiameter1 = i;
                indexDiameter2 = j;
            }
        }
    }
    
    extremeset.push_back(pointclouds[indexDiameter1]);
    extremeset.push_back(pointclouds[indexDiameter2]);
    
    extremeindex.insert(indexDiameter1);
    extremeindex.insert(indexDiameter2);
    
    auto it = find(remainningset.begin(),remainningset.end(), pointclouds[indexDiameter1]);
    if (it != remainningset.end()){
        remainningset.erase(it);
    }
    it = find(remainningset.begin(),remainningset.end(), pointclouds[indexDiameter2]);
    if (it != remainningset.end()){
        remainningset.erase(it);
    }
//    remainningset.erase(remainningset.begin()+indexDiameter1);
//    remainningset.erase(remainningset.begin()+indexDiameter2); // after the above deletion, this may be an error, since the size changed
    
    int count = number-2;
    double max = 0;
    int maxindex = 0;
    double min = diam;
    double distance = 0;
    double distanceE1X = 0;
    double distanceE2X = 0;
    double distanceE1E2 = 0;
    while(count > 0){
        max = 0;
        min = diam;
        maxindex = 0;
        long currentsize = extremeset.size();
        for (int x = 0; x < number; x++){
            if (extremeindex.find(x) == extremeindex.end()) { // not exist in extremeset
                for (int e1 = 0; e1 < currentsize; e1++){
                    for (int e2 = e1; e2 < currentsize; e2++){
                        if (e1 == e2) {
                            continue;
                        }
                        distanceE1X = sqrt((extremeset[e1][0]-pointclouds[x][0])*(extremeset[e1][0]-pointclouds[x][0]) +(extremeset[e1][1]-pointclouds[x][1])*(extremeset[e1][1]-pointclouds[x][1]) + (extremeset[e1][2]-pointclouds[x][2])*(extremeset[e1][2]-pointclouds[x][2]));
                        
                        distanceE2X = sqrt((extremeset[e2][0]-pointclouds[x][0])*(extremeset[e2][0]-pointclouds[x][0]) +(extremeset[e2][1]-pointclouds[x][1])*(extremeset[e2][1]-pointclouds[x][1]) + (extremeset[e2][2]-pointclouds[x][2])*(extremeset[e2][2]-pointclouds[x][2]));
                        
                        distanceE1E2 = sqrt((extremeset[e1][0]-extremeset[e2][0])*(extremeset[e1][0]-extremeset[e2][0]) +(extremeset[e1][1]-extremeset[e2][1])*(extremeset[e1][1]-extremeset[e2][1]) + (extremeset[e1][2]-extremeset[e2][2])*(extremeset[e1][2]-extremeset[e2][2]));
                        
                        distance = distanceE1X + distanceE2X - distanceE1E2;
                        
                        if (distance < min) {
                            min = distance;
                        }
                    }
                }
                
                if (min > max) {
                    max = min;
                    maxindex = x;
                }
            }
        }
        extremeset.push_back(pointclouds[maxindex]);
        extremeindex.insert(maxindex);
//        cout << "max: " << max << endl;
        
        it = find(remainningset.begin(),remainningset.end(), pointclouds[maxindex]);
        if (it != remainningset.end()){
            remainningset.erase(it);
        }
        count--;
    }
    
    extremesetAndRemainningset result;
    result.extremeset = extremeset;
    result.remainningset = remainningset;
    
    return result;
}

multimap<vector<double>,vector<double>> calculateExtremeSetRelation(const vector<vector<double>> extremeset1, const vector<vector<double>> extremeset2, double delta){
    
    multimap<vector<double>,vector<double>> relation;
    
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    IloObjective obj(env);
    IloNumVarArray vars(env);
    IloRangeArray ranges(env);
    
    double target = -1.0;
    long m = extremeset1.size();
    long n = extremeset2.size();
    long totalNum = m*n;
    
    // set variable type, IloNumVarArray starts from 0.
    for (int i = 0; i < totalNum; i++){
        vars.add(IloNumVar(env, 0.0, 1.0, ILOINT));
    }
    
    // declare objective
    for (int i = 0; i < totalNum; i++){
        obj.setLinearCoef(vars[i], 1.0);
    }
    obj.setSense(IloObjective::Maximize);
    model.add(obj);
    
    // add constraint
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
                    denominator = calculateDistortion(i, j, k, l, extremeset1, extremeset2);
                    if (denominator == 0){
                        model.add(vars[i*n+k] + vars[j*n+l] <= 2);
                    } else {
                        model.add(vars[i*n+k] + vars[j*n+l] -(delta/denominator) <= 1);
                    }
                }
            }
        }
    }
    
    if(!cplex.solve()){
        cout << "cplex solve failure ! " << endl;
    }
    target = cplex.getObjValue();
//    cout << "target : " << target << endl;
    
    int resulti;
    vector<double> tempx;
    vector<double> tempy;
    int xindex = 0;
    int yindex = 0;
    for (int i = 0; i < totalNum; i++){
        resulti = cplex.getValue(vars[i]);
        if(resulti == 1){
            xindex = i / n;
            yindex = i % n;
            relation.insert(pair<vector<double>,vector<double>>(extremeset1[xindex], extremeset2[yindex]));
        }
    }
    
    env.end();
    return relation;
}

multimap<vector<double>,vector<double>> incrementExtendCorrespondence(vector<vector<double>> extremeset1, vector<vector<double>> extremeset2, vector<vector<double>> referenceset1,  vector<vector<double>> referenceset2, multimap<vector<double>,vector<double>> relation){
    
    long extremeSize = extremeset1.size();
    long referenceSize = referenceset1.size();
    long fullreferenceSize = extremeSize + referenceSize;
    
    vector<vector<double>> fullreferenceset1(extremeset1);
    vector<vector<double>> fullreferenceset2(extremeset2);
    
    fullreferenceset1.insert(fullreferenceset1.end(), referenceset1.begin(), referenceset1.end());
    fullreferenceset2.insert(fullreferenceset2.end(), referenceset2.begin(), referenceset2.end());
    
    double max = 0;
    double min = 0;
    auto miniterator = fullreferenceset2.begin();
    
    
    bool firsttry = true;
    double tempdistance = 0;
    
    vector<double> tempx;
    vector<double> tempy;
    vector<double> tempxp;
    vector<double> tempyp;
    
    // add the x in extremeset that are not in relation to referenceset
    vector<double> temp;
    for (auto it = extremeset1.begin(); it != extremeset1.end(); it++){
        temp = *it;
        auto itr = relation.find(temp);
        if (itr == relation.end()) { // not in relation
            referenceset1.push_back(temp);
        }
    }
    
    // for x in X\P1(R)
    for(auto itx = referenceset1.begin(); itx != referenceset1.end(); itx++){ // 这里要改， 这个并不是 X - relation中x的，可能还有extremeset中的一些！
        tempx = *itx;
        for (auto ity = fullreferenceset2.begin(); ity != fullreferenceset2.end(); ity++){
            tempy = *ity;
            for (auto itr = relation.begin(); itr != relation.end(); itr++){
                tempxp = itr->first;
                tempyp = itr->second;
                tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
                if(tempdistance > max){
                    max = tempdistance;
                    if (firsttry){
                        min = max;
                        firsttry = false;
                    }
                }
            }
            if (max < min){
                min = max;
                miniterator = ity;
            }
        }
        
        // calculate c
        // wait for implement
        
        relation.insert(pair<vector<double>,vector<double>>(*itx, *miniterator));
        vector<double> lala = *miniterator;
        auto deletepointer = find(referenceset2.begin(),referenceset2.end(), lala); // 找不到？？？
        if (deletepointer != referenceset2.end()){
            referenceset2.erase(deletepointer);
            cout << "================= delete success" << endl;
//            referenceset2.erase(miniterator);  // miniterator is belong to fullreferenceset2!!!
        }
    }
    
    firsttry = true;
    max = 0;
    min = 0;
    
    // add the y in extremeset that are not in relation to referenceset
//    vector<double> temp;
    bool flag = false;
    for (auto it = extremeset2.begin(); it != extremeset2.end(); it++){
        temp = *it;
        flag = false;
        for (auto itr = relation.begin(); itr != relation.end(); itr++){
            if (itr->second == temp){
                flag = true;
            }
        }
        if (flag) { // not in relation
            referenceset2.push_back(temp);
        }
    }
    
    // for y in Y\P2(R)
    for(auto ity = referenceset2.begin(); ity != referenceset2.end(); ity++){
        tempy = *ity;
        for (auto itx = fullreferenceset1.begin(); itx != fullreferenceset1.end(); itx++){
            tempx = *itx;
            for (auto itr = relation.begin(); itr != relation.end(); itr++){
                tempxp = itr->first;
                tempyp = itr->second;
                tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
                if(tempdistance > max){
                    max = tempdistance;
                    if (firsttry){
                        min = max;
                        firsttry = false;
                    }
                }
            }
            if (max < min){
                min = max;
                miniterator = itx;
            }
        }
        
        // calculate c
        // wait for implement
        
        relation.insert(pair<vector<double>,vector<double>>(*miniterator, *ity));
//        referenceset1.erase(miniterator); // no need to do this any more
    }
    
    return relation;
}

multimap<vector<double>,vector<double>> extendCorrespondence(vector<vector<double>> pointcloud1, vector<vector<double>> pointcloud2, multimap<vector<double>,vector<double>> relation){
    
    vector<vector<double>> remainningset1;
    vector<vector<double>> remainningset2;
    vector<double> tempx;
    vector<double> tempy;
    for (auto itr = relation.begin(); itr != relation.end(); itr++){
        tempx = itr->first;
        tempy = itr->second;
        if (find(remainningset1.begin(), remainningset1.end(), tempx) == remainningset1.end()){
            remainningset1.push_back(tempx);
        }
        if (find(remainningset2.begin(), remainningset2.end(), tempy) == remainningset2.end()){
            remainningset2.push_back(tempy);
        }
    }
    
    vector<double> tempxp;
    vector<double> tempyp;
    
    double max = 0;
    double min = 0;
    auto miniterator = remainningset2.begin();
    
    bool firsttry = true;
    double tempdistance = 0;
    
    for(auto itx = remainningset1.begin(); itx != remainningset1.end(); itx++){
        tempx = *itx;
        for (auto ity = remainningset2.begin(); ity != remainningset2.end(); ity++){
            tempy = *ity;
            for (auto itr = relation.begin(); itr != relation.end(); itr++){
                tempxp = itr->first;
                tempyp = itr->second;
                tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
                if(tempdistance > max){
                    max = tempdistance;
                    if (firsttry){
                        min = max;
                        miniterator = ity;
                        firsttry = false;
                    }
                }
            }
            if (max < min){
                min = max;
                miniterator = ity;
            }
        }
        
        // calculate c
        // wait for implement
        if (itx != remainningset1.end() && miniterator != remainningset2.end()){
            relation.insert(pair<vector<double>,vector<double>>(*itx, *miniterator));
        }
        vector<double> lala = *miniterator;
        auto deletepointer = find(remainningset2.begin(),remainningset2.end(), lala); // 找不到？？？
        if (deletepointer != remainningset2.end()){
            remainningset2.erase(deletepointer);
//            cout << "================= delete success" << endl;
            //            referenceset2.erase(miniterator);  // miniterator is belong to fullreferenceset2!!!
        }
    }
    
    firsttry = true;
    max = 0;
    min = 0;
    
    for(auto ity = remainningset2.begin(); ity != remainningset2.end(); ity++){
        tempy = *ity;
        for (auto itx = remainningset1.begin(); itx != remainningset1.end(); itx++){
            tempx = *itx;
            for (auto itr = relation.begin(); itr != relation.end(); itr++){
                tempxp = itr->first;
                tempyp = itr->second;
                tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
                if(tempdistance > max){
                    max = tempdistance;
                    if (firsttry){
                        min = max;
                        miniterator = itx;
                        firsttry = false;
                    }
                }
            }
            if (max < min){
                min = max;
                miniterator = itx;
            }
        }
        
        // calculate c
        // wait for implement
        
        if (miniterator != remainningset1.end() && ity != remainningset2.end()){
            relation.insert(pair<vector<double>,vector<double>>(*miniterator, *ity));
            //        referenceset1.erase(miniterator); // no need to do this any more
        }
    }
    
    return relation;
}

multimap<vector<double>,vector<double>> improveCorrespondence(const vector<vector<double>> pointcloud1, const vector<vector<double>> pointcloud2, multimap<vector<double>,vector<double>> relation, double tolerance){
    
    double tempdistance;
    double innermax = 0;
    double outermax = 0;
    
    vector<double> tempx;
    vector<double> tempy;
    vector<double> tempxp;
    vector<double> tempyp;
    
    auto maxiterator = relation.begin();
    
    vector<double> maxx;
    vector<double> maxy;
    
    double distortionDifference = 0;
    
    multimap<vector<double>,vector<double>> tempRelation;
    
    do {
        
        innermax = 0;
        outermax = 0;
        
        tempRelation = relation;
        
        for (auto itr1 = relation.begin(); itr1 != relation.end(); itr1++){
            tempx = itr1->first;
            tempy = itr1->second;
            for (auto itr2 = itr1; itr2 != relation.end(); itr2++){
                if (itr1 == itr2){
                    continue;
                }
                tempxp = itr2->first;
                tempyp = itr2->second;
                
                tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
                
                if (tempdistance > innermax){
                    innermax = tempdistance;
                }
            }
            if (innermax > outermax){
                outermax = innermax;
                maxiterator = itr1;
                maxx = tempx;
                maxy = tempy;
            }
        }
        relation.erase(maxiterator);
        
        
        double max = 0;
        double min = 0;
        auto miniterator = pointcloud2.begin();
        double firstflag = true;
        
        // find maxx still in relation
        
        if (relation.find(maxx) == relation.end()){ // if x do not exist in relation anymore
            // build a new relation for x
            // find the min y
            tempx = maxx;
            for (auto ity = pointcloud2.begin(); ity != pointcloud2.end(); ity++){
                tempy = *ity;
                for (auto itr = relation.begin(); itr != relation.end(); itr++){
                    tempxp = itr->first;
                    tempyp = itr->second;
                    tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
                    if (tempdistance > max){
                        max = tempdistance;
                        if (firstflag){
                            min = max;
                            firstflag = false;
                        }
                    }
                }
                if (max < min){
                    min = max;
                    miniterator = ity;
                }
            }
            relation.insert(pair<vector<double>,vector<double>>(maxx, *miniterator));
        }
        
        max = 0;
        min = 0;
        firstflag = true;
        miniterator = pointcloud1.begin();
        
        // find maxy still in relation?
        bool flag = false;
        for (auto itr = relation.begin(); itr != relation.end(); itr++){
            if (itr->second == maxy){
                flag = true;
            }
        }
        if (!flag){ // if y do not exist in relation anymore
            // build a new relation for y
            // find the min x
            tempy = maxy;
            for (auto itx = pointcloud1.begin(); itx != pointcloud1.end(); itx++){
                tempx = *itx;
                for (auto itr = relation.begin(); itr != relation.end(); itr++){
                    tempxp = itr->first;
                    tempyp = itr->second;
                    tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
                    if (tempdistance > max){
                        max = tempdistance;
                        if (firstflag){
                            min = max;
                            firstflag = false;
                        }
                    }
                }
                if (max < min){
                    min = max;
                    miniterator = itx;
                }
            }
            relation.insert(pair<vector<double>,vector<double>>(*miniterator, maxy));
        }
        
        distortionDifference = calculateDistortionDifference(tempRelation, relation);
        if (distortionDifference < 0){
            relation = tempRelation;
        }
        
    } while(distortionDifference <= tolerance);
    
    return relation;
}

// relation1 should be R (tempRelation above), relation2 should be RI (relation above)
double calculateDistortionDifference(multimap<vector<double>,vector<double>> relation1, multimap<vector<double>,vector<double>> relation2){
    
    double difference = 0;
    double distortion1 = 0;
    double distortion2 = 0;
    
    vector<double> tempx;
    vector<double> tempy;
    vector<double> tempxp;
    vector<double> tempyp;
    
    double tempdistance;
    double max = 0;
    
    for (auto itr1 = relation1.begin(); itr1 != relation1.end(); itr1++){
        tempx = itr1->first;
        tempy = itr1->second;
        for (auto itr2 = itr1; itr2 != relation1.end(); itr2++){
            if (itr1 == itr2){
                continue;
            }
            tempxp = itr2->first;
            tempyp = itr2->second;
            
            tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
            
            if (tempdistance > max){
                max = tempdistance;
            }
        }
    }
    distortion1 = max;
    
    max = 0;
    
    for (auto itr1 = relation2.begin(); itr1 != relation2.end(); itr1++){
        tempx = itr1->first;
        tempy = itr1->second;
        for (auto itr2 = itr1; itr2 != relation2.end(); itr2++){
            if (itr1 == itr2){
                continue;
            }
            tempxp = itr2->first;
            tempyp = itr2->second;
            
            tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
            
            if (tempdistance > max){
                max = tempdistance;
            }
        }
    }
    distortion2 = max;
    
    difference = distortion1 - distortion2;
    
    return difference;
}

double calculateRelationDistortion(multimap<vector<double>,vector<double>> relation){
    
    double distortion = 0;
    
    vector<double> tempx;
    vector<double> tempy;
    vector<double> tempxp;
    vector<double> tempyp;
    
    double tempdistance;
    
    for (auto itr1 = relation.begin(); itr1 != relation.end(); itr1++){
        tempx = itr1->first;
        tempy = itr1->second;
        for (auto itr2 = itr1; itr2 != relation.end(); itr2++){
            if (itr1 == itr2){
                continue;
            }
            tempxp = itr2->first;
            tempyp = itr2->second;
            
            tempdistance = fabs(sqrt((tempx[0] - tempxp[0])*(tempx[0] - tempxp[0]) + (tempx[1] - tempxp[1])*(tempx[1] - tempxp[1]) + (tempx[2] - tempxp[2])*(tempx[2] - tempxp[2])) - sqrt((tempy[0] - tempyp[0])*(tempy[0] - tempyp[0]) + (tempy[1] - tempyp[1])*(tempy[1] - tempyp[1]) + (tempy[2] - tempyp[2])*(tempy[2] - tempyp[2])));
            
            if (tempdistance > distortion){
                distortion = tempdistance;
            }
        }
    }
    
    return distortion;
}
