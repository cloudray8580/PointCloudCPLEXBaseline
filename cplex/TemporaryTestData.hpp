//
//  TemporaryTestData.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/9.
//  Copyright © 2017年 李喆. All rights reserved.
//

#ifndef TemporaryTestData_hpp
#define TemporaryTestData_hpp

#include <stdio.h>
#endif /* TemporaryTestData_hpp */

vector<vector<double>> pointclouds1 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test1.pts"); // cube with length 1 in quadrant 1
vector<vector<double>> pointclouds2 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test2.pts"); // cube with length 1 in quadrant 7
vector<vector<double>> pointclouds3 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test3.pts"); // cube with length 2 in quadrant 1
vector<vector<double>> pointclouds4 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test4.pts"); // 2D rectangle 2*4
vector<vector<double>> pointclouds5 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test5.pts"); // 2D rectangle 2*2
vector<vector<double>> pointclouds6 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test6.pts"); // 3D Up Centrum 5 points
vector<vector<double>> pointclouds7 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test7.pts"); // 3D Down Centrum 5 points
vector<vector<double>> pointclouds8 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test8.pts"); // 2D square with length 1
vector<vector<double>> pointclouds9 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test9.pts"); // 2D square with length 2
vector<vector<double>> pointclouds10 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test10.pts"); // line with length 4
vector<vector<double>> pointclouds11 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test11.pts"); // line with length 2
vector<vector<double>> pointclouds12 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test12.pts"); // cube with length 4 20 points
vector<vector<double>> pointclouds13 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/test13.pts"); // cube with length 2 20 points
vector<vector<double>> pointcloudsX1 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/000020.pts"); // totally 2639 points
vector<vector<double>> pointcloudsX2 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/000907.pts"); // totally 2518 points
vector<vector<double>> pointcloudsX3 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/000062.pts"); // totally 2708 points
vector<vector<double>> pointcloudsX4 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/000640.pts"); // totally 2687 points
vector<vector<double>> pointcloudsX5 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/Nonnormalized1.pts"); // non normalized dataset, 420 points
vector<vector<double>> pointcloudsX6 = creatPointCloudArrayFromFile("/Users/lizhe/Desktop/pointclouddataset/other/Nonnormalized2.pts"); // non normalized dataset, 722 points

vector<vector<double>> _10ofX1 = randomExtract(pointcloudsX1, 10);
vector<vector<double>> _10ofX2 = randomExtract(pointcloudsX2, 10);

vector<vector<double>> _20ofX1 = randomExtract(pointcloudsX1, 20);
vector<vector<double>> _20ofX2 = randomExtract(pointcloudsX2, 20);

vector<vector<double>> _25ofX1 = randomExtract(pointcloudsX1, 25);
vector<vector<double>> _25ofX2 = randomExtract(pointcloudsX2, 25);

vector<vector<double>> _30ofX1 = randomExtract(pointcloudsX1, 30);
vector<vector<double>> _30ofX2 = randomExtract(pointcloudsX2, 30);

vector<vector<double>> _40ofX1 = randomExtract(pointcloudsX1, 40);
vector<vector<double>> _40ofX2 = randomExtract(pointcloudsX2, 40);

vector<vector<double>> _50ofX1 = randomExtract(pointcloudsX1, 50);
vector<vector<double>> _50ofX2 = randomExtract(pointcloudsX2, 50);

vector<vector<double>> _100ofX1 = randomExtract(pointcloudsX1, 100);
vector<vector<double>> _100ofX2 = randomExtract(pointcloudsX2, 100);

vector<vector<double>> _200ofX1 = randomExtract(pointcloudsX1, 200);
vector<vector<double>> _200ofX2 = randomExtract(pointcloudsX2, 200);

vector<vector<double>> _500ofX1 = randomExtract(pointcloudsX1, 500);
vector<vector<double>> _500ofX2 = randomExtract(pointcloudsX2, 500);

vector<vector<double>> _1000ofX1 = randomExtract(pointcloudsX1, 1000);
vector<vector<double>> _1000ofX2 = randomExtract(pointcloudsX2, 1000);
