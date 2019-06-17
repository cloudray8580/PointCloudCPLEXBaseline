//
//  MaxLoss.hpp
//  cplex
//
//  Created by 李喆 on 2019/6/13.
//  Copyright © 2019 李喆. All rights reserved.
//
#pragma once
#ifndef MaxLoss_hpp
#define MaxLoss_hpp

#include <stdio.h>
#include <armadillo>
#include <vector>
#include <ilcplex/ilocplex.h>
#include <chrono>

using namespace std;

double myCplexSolveForMaxLoss(vector<double> &keys, vector<int> &pos){
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(model);
    IloObjective obj(env);
    IloNumVarArray vars(env);
    IloRangeArray ranges(env);
    
    // set variable type, IloNumVarArray starts from 0.
    vars.add(IloNumVar(env, 0.0, INFINITY, ILOFLOAT)); // the weight, i.e., a
    vars.add(IloNumVar(env, -INFINITY, INFINITY, ILOFLOAT));      // the bias, i.e., b
    vars.add(IloNumVar(env, 0.0, INFINITY, ILOFLOAT)); // our target, the max loss
    
    // declare objective
    obj.setExpr(vars[2]);
    obj.setSense(IloObjective::Minimize);
    model.add(obj);
    
    // add constraint for each record
    for(int i = 0; i < keys.size(); i++){
        model.add(vars[0]*keys[i] + vars[1] - pos[i] <= vars[2]);
        model.add(vars[0]*keys[i] + vars[1] - pos[i] >= -vars[2]);
    }
    
    if(!cplex.solve()){
        cout << "cplex solve failure ! " << endl;
    }
    double target = cplex.getObjValue();
    
    cout << "the variable a: " << cplex.getValue(vars[0]) << endl;
    cout << "the variable b: " << cplex.getValue(vars[1]) << endl;
    cout << "the variable max loss: " << cplex.getValue(vars[2]) << endl;
    
    env.end();
    return target;
}


void call_linear_programming(){
    arma::mat dataset;
    dataset.load("/Users/lizhe/Library/Mobile Documents/com~apple~CloudDocs/SortedSingleDimPOIs2.csv");
    
    arma::colvec key_ = dataset.col(0);
    arma::colvec pos_ = dataset.col(dataset.n_cols-1);
    
    vector<double> keys;
    vector<int> pos;
    
    for(int i = 0; i < key_.n_rows; i++){
        keys.push_back(key_[i]);
        pos.push_back(pos_[i]);
    }
    
    auto t0 = chrono::steady_clock::now();
    myCplexSolveForMaxLoss(keys, pos);
    auto t1 = chrono::steady_clock::now();
    cout << "Total Time in chrono: " << chrono::duration_cast<chrono::nanoseconds>(t1 - t0).count() << " ns" << endl;

}

#endif /* MaxLoss_hpp */
