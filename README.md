# FIxed-Wing-Aerial-TrajOpt
Repository associated with IROS 2021 submission " Embedded Hardware Appropriate Fast 3D Trajectory Optimization for Fixed Wing Aerial Vehicles  by Levereging Hidden Convex structures"

Codes will start appearing here from Monday 14th March  

### Running our custom optimizer
* Navigate to the ```ourOptimizer``` folder and edit configuration details in ```config.yaml``` file.
* Run the following command ```g++ main.cpp FWV_optim.cpp $(pkg-config --cflags --libs yaml-cpp) -o test -O2```
