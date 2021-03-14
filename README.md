# FIxed-Wing-Aerial-TrajOpt
Repository associated with IROS 2021 submission " Embedded Hardware Appropriate Fast 3D Trajectory Optimization for Fixed Wing Aerial Vehicles  by Levereging Hidden Convex structures"

Codes will start appearing here from Monday 14th March  

### Running our custom optimizer
* Navigate to the ```ourOptimizer``` folder and edit configuration details in ```config.yaml``` file.
* Run the following command ```g++ main.cpp FWV_optim.cpp $(pkg-config --cflags --libs yaml-cpp) -o test -O2``` followed by ```./test```  


### Running generated ACADO code
* Navigate to any of the config folder and run ```make clean all``` followed by ```./test```.   
_Note: Only for the first time do_ ```make clean all``` _otherwise just_ ```make```
 
