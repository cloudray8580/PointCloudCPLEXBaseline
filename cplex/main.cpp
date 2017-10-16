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
#include <math.h>
#include <string>
using namespace std;

double myCplexSolve(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2);
double calculateRotation(const int i, const int j, const int k, const int l, const vector<vector<double>> &pointclouds1,
                         const vector<vector<double>> &pointclouds2);
vector<vector<double>> creatPointCloudArrayFromFile(string filename);
void printVector(vector<vector<double>> pointclouds);

int main(int argc, const char * argv[]) {
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//    IloObjective obj(env);
//    IloNumVarArray x(env);
//    IloRangeArray rngs(env);
    
//    const char filename[] = "/Users/lizhe/Desktop/pointclouddataset/testlp42and22.lp"; // the lp_solve generated .lp file do not work
//    const char filename[] = "/Users/lizhe/Desktop/pointclouddataset/testlp42and22.mps"; // use the .mps file instead should be OK
//    cplex.importModel(model, filename);
//    cplex.importModel(model, filename, obj, x, rngs);
    
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
    
    time_t start,stop;
    start = time(NULL);
    double target = myCplexSolve(pointclouds1, pointclouds3);
    stop = time(NULL);

    cout << "==========================================" << endl;
    cout << "  our target : " << target << "   ----   time usage : "<< stop-start << endl;
    cout << "==========================================" << endl;
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
                        continue;
                    }
                    denominator = calculateRotation(i, j, k, l, pointclouds1, pointclouds2);
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
    env.end();
    return target;
}

double calculateRotation(const int i, const int j, const int k, const int l, const vector<vector<double>> &pointclouds1,
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
