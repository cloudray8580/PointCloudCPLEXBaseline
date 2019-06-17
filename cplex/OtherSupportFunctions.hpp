//
//  OtherSupportFunctions.hpp
//  cplex
//
//  Created by 李喆 on 2017/11/9.
//  Copyright © 2017年 李喆. All rights reserved.
//

#ifndef OtherSupportFunctions_hpp
#define OtherSupportFunctions_hpp

#include <stdio.h>
#include <dirent.h>

#endif /* OtherSupportFunctions_hpp */

void printFileInDir(string dirpath){
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (dirpath.c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            printf ("%s\n", ent->d_name);
        }
        closedir (dir);
    } else {
        /* could not open directory */
        perror ("");
        printf ("%s\n", "could not open dir");
    }
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
    if (num <= 0){
        return subpointcloud;
    }
    
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

//double runSolution(vector<vector<double>> &pointclouds1, vector<vector<double>> &pointclouds2){
//    time_t start,stop;
//    start = time(NULL);
//    double target = myCplexSolve(pointclouds1, pointclouds2); // call to my CPLEX related function
//    stop = time(NULL);
//
//    cout << "====================================================" << endl;
//    cout << "  GHD : " << target / 2 << "   ----   time usage : "<< stop-start << endl;
//    cout << "====================================================" << endl;
//
//    return target;
//}
//
//double runSolutionThroughFile(const char filename[]){
//    IloEnv env;
//    IloModel model(env);
//    IloCplex cplex(model);
//    IloObjective obj(env);
//    IloNumVarArray x(env);
//    IloRangeArray rngs(env);
//
//    // the lp_solve generated .lp file do not work
//    // use the .mps file instead should be OK
//    cplex.importModel(model, filename, obj, x, rngs);
//
//    time_t start,stop;
//    start = time(NULL);
//    cplex.solve();
//    stop = time(NULL);
//
//    double target = cplex.getObjValue();
//
//    cout << "==========================================" << endl;
//    cout << "  our target : " << target << "   ----   time usage : "<< stop-start << endl;
//    cout << "==========================================" << endl;
//
//    env.end();
//    return target;
//}

