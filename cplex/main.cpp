//
//  main.cpp
//  cplex
//
//  Created by 李喆 on 2017/10/12.
//  Copyright © 2017年 李喆. All rights reserved.
//

#include <iostream>
#include "TestManagement.hpp"
#include "TemporaryTestData.hpp"

using namespace std;

int main(int argc, const char * argv[]) {

//    TestManagement tm("/Users/lizhe/Desktop/plotdata/test.cfg");
//    tm.run();
    
    PointCloud p1(pointclouds12);
    PointCloud p2(pointclouds13);
    PointCloudsCalculation pcc;
    double ghd = pcc.GHDistance_CPLEX(p1, p2);
    double ub1 = pcc.Upperbound_UB1(p1, p2);
    double ub3 = pcc.Upperbound_UB3(p1, p2, 6, 0.5, 12, 0.00001);
    double lb1 = pcc.Lowerbound_LB1(p1, p2);
    double lb4 = pcc.Lowerbound_LB4(p1, p2);
    cout << "ghd: " << ghd << endl;
    cout << "ub1: " << ub1 << endl;
    cout << "ub3: " << ub3 << endl;
    cout << "lb1: " << lb1 << endl;
    cout << "lb4: " << lb4 << endl;
}
