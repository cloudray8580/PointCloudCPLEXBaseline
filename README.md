# PointCloudCPLEXBaseline

This project aims at using Gromov-Hausdorrf distance(GHD) to calculate the KNN of Point Cloud data with pratical running time.
We provide the following functions:

1. Using BILP to calculate the exact GHD with IBM CPLEX library, but the running time increase very fast as number of points increase
 so, the maximum of point should not larger than 20.
 
2. Provides some baseline upperbound and lowerbound algorithm implementation.

3. Provides our own upperbound and lowerbound calculation algorithm implementation.



To use, you need to change the configuration file, and start use like this:

  TestManagement tm("/Users/lizhe/Desktop/plotdata/test.cfg");
  tm.run();
