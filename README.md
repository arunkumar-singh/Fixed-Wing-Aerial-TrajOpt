# FIxed-Wing-Aerial-TrajOpt
Repository associated with IROS 2021 submission " Embedded Hardware Appropriate Fast 3D Trajectory Optimization for Fixed Wing Aerial Vehicles  by Levereging Hidden Convex structures"

Codes will start appearing here from Monday 14th March  

### Running our custom optimizer
* Navigate to the ```ourOptimizer``` folder and edit configuration details in ```config.yaml``` file.
* Run the following command ```g++ main.cpp FWV_optim.cpp $(pkg-config --cflags --libs yaml-cpp) -o test -O2``` followed by ```./test```  


### Running generated ACADO code
* Navigate to any of the config folder and run ```make clean all``` followed by ```./test```.   
_Note: Only for the first time do_ ```make clean all``` _otherwise just_ ```make```
 
### Configuration taken for Comparison     
| No. |x | y | z | psi | xg | yg | zg | planning time |  
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |  
|1|	5	|130|	30	|0|	120|	40|	10|	13.8|  
|2|	5|	130|	30|	0|	120|	40|	10|	13.8|
|3|5|	130|	30|	-0.2|	120|	40|	10|	13.8|
|4|	5|	130|	30|	-0.2|	120|	40|	10|	13.7|
|5|	-30|	-10	|30	|0.2	|130	|138	|40	|18.3|
|6|	-30|	-10|	30|	0.2|	130|	138|	40|	18.3|
|7|	-30	|-10	|30	|0	|130	|138	|40	|19.3|  
|8|	-30|	-10|	30|	0|	130|	138|	40|	19.3|
|9|	-30	|-10	|30	|0.33	|130	|138	|40	|18.4|
|10|	-30|	-10|	30|	0.33|	130|	138|	40|	18.3|
|11|-30	|-10	|30	|0.5	|130	|138	|40	|18.6| 
|12|	-30|	-10|	30|	0.5|	130|	138|	40|	18.6| 
|13|	-30	|-10	|30	|-0.2	|130	|138	|40	|19.5|
|14|	-30|	-10|	30|	-0.2|	130|	138|	40|	19.5|
|15|5	|130|	30|	0.2	|120|	40|	10	|13.9|  
|16	|5	|130	|30	|0.2	|120	|40	|10	|14|
|17|5	|130	|30|	-0.33|	120|	40	|10	|12.9|  
|18|	5	|130|	30|	-0.33|	120|	40|	10|	12.9|
|19|5	|130	|30	|-0.45	|120	|40	|10	|13.4|  
|20	|5	|130	|30	|-0.45	|120	|40	|10	|13.4|   



50 and 100 steps of each configuration












