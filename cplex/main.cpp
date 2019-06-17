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
#include "MaxLoss.hpp"
using namespace std;

int main(int argc, const char * argv[]) {
    
    call_linear_programming();
    
//    TestManagement tm("runconfig.cfg");
//    tm.run();
    
////    PointCloud p1(_30ofNonnor1);
////    PointCloud p2(_30ofNonnor2);
//    PointCloud p1(pointcloudsX5);
//    PointCloud p2(pointcloudsX6);
//
//
//    string filename;
//    ofstream outfile;
//    filename = "/Users/lizhe/Desktop/plotdata/testformyupperbound";
//    outfile.open(filename);
//
//    PointCloudsCalculation pcc;
////    double ghd = pcc.GHDistance_CPLEX(p1, p2);
//    PointCloud::clusterResult results1;
//    PointCloud::clusterResult results2;
//    double ghdxcyc = 0;
//    double maxradius1 = 0;
//    double maxradius2 = 0;
//    double maxradius = 0;
//    double ub;
//    double lb;
//    double ub1 = pcc.Upperbound_UB1(p1, p2);
//    double lb1 = pcc.Lowerbound_LB1(p1, p2);
//    for(int k = 2; k <= 30; k++){
//        results1 = p1.runKcenters3(k);
//        results2 = p2.runKcenters3(k);
//        ghdxcyc = pcc.GHDistance_CPLEX(results1.centers, results2.centers);
//        maxradius1 = PointCloud::calculateClusterMaxRadius(results1.clusters);
//        maxradius2 = PointCloud::calculateClusterMaxRadius(results2.clusters);
//        maxradius = maxradius1 >= maxradius2 ? maxradius1 : maxradius2;
//        ub = ghdxcyc + maxradius;
//        lb = ghdxcyc - maxradius;
//
//        PointCloudsCalculation::LB_UB bounds = pcc.LowerboundAndUpperbound_kcenters(k, p1, p2);
////        outfile << k << " " << ghd << " " << ub << " " << lb << " " << ghdxcyc << " " << bounds.upperbound << " " << bounds.lowerbound << endl;
//        outfile << k << " " << ub << " " << lb << " " << ghdxcyc << " " << bounds.upperbound << " " << bounds.lowerbound << " " << ub1 << " " << lb1 << endl;
//    }
//    outfile.close();
//

    
    
//    PointCloudsCalculation pcc;
//    double ghd = pcc.GHDistance_CPLEX(p1, p2);
//    double ub1 = pcc.Upperbound_UB1(p1, p2);
//    double ub3 = pcc.Upperbound_UB3(p1, p2, 6, 0.5, 12, 0.00001);
//    double lb1 = pcc.Lowerbound_LB1(p1, p2);
//    double lb4 = pcc.Lowerbound_LB4(p1, p2);
//    cout << "ghd: " << ghd << endl;
//    cout << "ub1: " << ub1 << endl;
//    cout << "ub3: " << ub3 << endl;
//    cout << "lb1: " << lb1 << endl;
//    cout << "lb4: " << lb4 << endl;
}
