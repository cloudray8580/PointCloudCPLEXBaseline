//
//  Lowerbound4.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/9.
//  Copyright © 2017年 李喆. All rights reserved.
//

#ifndef Lowerbound4_hpp
#define Lowerbound4_hpp

#include <stdio.h>

#endif /* Lowerbound4_hpp */

double myCplexSolveForLB4_1(vector<vector<double>> pointclouds1, vector<vector<double>> pointclouds2);

double myCplexSolveForLB4_2(vector<vector<double>> pointclouds1, vector<vector<double>> pointclouds2);




// 1/2 max (infdis(x->y), infdis(y->x))
double calculateLowerBound4(const vector<vector<double>> pointclouds1, const vector<vector<double>> pointclouds2){
    
    time_t start,stop;
    start = time(NULL);
    
    double lowerbound = 0;
    
    double infdisx2y = myCplexSolveForLB4_1(pointclouds1, pointclouds2);
    double infdisy2x = myCplexSolveForLB4_2(pointclouds1, pointclouds2);
    
    lowerbound = infdisx2y > infdisy2x ? infdisx2y : infdisy2x;
    lowerbound /= 2;
    
    stop = time(NULL);
    
    cout << "=================================================" << endl;
    cout << "  Lowerbound4 : " << lowerbound << "   time : " << stop-start << endl;
    cout << "  infdisx->y : " << infdisx2y << endl;
    cout << "  infdisy->x : " << infdisy2x << endl;
    cout << "=================================================" << endl;
    
    return lowerbound;
}

// used for method calculateLowerBound4
double myCplexSolveForLB4_1(vector<vector<double>> pointclouds1, vector<vector<double>> pointclouds2){
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
    
    // add constraint for formula 1, notice mapping is not relation, one y for each x
    for (int i = 0; i < m; i++){
        IloNumVarArray tempVars(env,n);
        IloNumArray tempCoefs(env,n);
        IloRange tempRange(env,1.0,1.0); // expression = 1.0
        for (int k = 0; k < n; k++){
            tempRange.setLinearCoef(vars[i*n+k], 1.0);
        }
        model.add(tempRange);
    }
    
    //    // add constraint for formual 2
    //    for (int k = 0; k < n; k++){
    //        IloNumVarArray tempVars(env,m);
    //        IloNumArray tempCoefs(env,m);
    //        IloRange tempRange(env,1.0,INFINITY);
    //        for (int i = 0; i < m; i++){
    //            tempRange.setLinearCoef(vars[i*n+k], 1.0);
    //        }
    //        model.add(tempRange);
    //    }
    
    
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

// used for method calculateLowerBound4
double myCplexSolveForLB4_2(vector<vector<double>> pointclouds1, vector<vector<double>> pointclouds2){
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
    
    // add constraint for formula 1, notice mapping is not relation, one y for each x
    //    for (int i = 0; i < m; i++){
    //        IloNumVarArray tempVars(env,n);
    //        IloNumArray tempCoefs(env,n);
    //        IloRange tempRange(env,1.0,1.0); // expression = 1.0
    //        for (int k = 0; k < n; k++){
    //            tempRange.setLinearCoef(vars[i*n+k], 1.0);
    //        }
    //        model.add(tempRange);
    //    }
    
    // add constraint for formual 2, notice mapping is not relation, one x for each y
    for (int k = 0; k < n; k++){
        IloNumVarArray tempVars(env,m);
        IloNumArray tempCoefs(env,m);
        IloRange tempRange(env,1.0,1.0); // expression = 1.0
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

