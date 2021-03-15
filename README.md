# Fixed-Wing-Aerial-TrajOpt
Repository associated with IROS 2021 submission " Embedded Hardware Appropriate Fast 3D Trajectory Optimization for Fixed Wing Aerial Vehicles  by Levereging Hidden Convex structures"

Codes will start appearing here from Monday 14th March  

### Running our custom optimizer
* Navigate to the ```ourOptimizer_50steps``` or ```ourOptimizer_100steps``` folder and edit configuration details in ```config.yaml``` file.
* In ```config.yaml``` file, entering ```psi_init``` as ```0.2``` indicates ```0.2pi``` in main code.
* Run the following command ```g++ main.cpp FWV_optim.cpp $(pkg-config --cflags --libs yaml-cpp) -o test -O2``` followed by ```./test```    
  
How to change steps (Edit global variable ```num``` in ```main.cpp``` file)

### Running generated ACADO code
* Navigate to any of the config folder and run ```make clean all``` followed by ```./test```.   
_Note: Only for the first time do_ ```make clean all``` _otherwise just_ ```make```
 
### Configuration taken for Comparison     
|Config No. |x | y | z | psi (\*pi) | xg | yg | zg | planning time | steps | 
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |:---: |  
|1|	5	|130|	30	|0|	120|	40|	10|	13.8| 50| 
|2|	5|	130|	30|	0|	120|	40|	10|	13.8|100|
|3|5|	130|	30|	-0.2|	120|	40|	10|	13.8|50|
|4|	5|	130|	30|	-0.2|	120|	40|	10|	13.7|100|
|5|	-30|	-10	|30	|0.2	|130	|138	|40	|18.3|50|
|6|	-30|	-10|	30|	0.2|	130|	138|	40|	18.3|100|
|7|	-30	|-10	|30	|0	|130	|138	|40	|19.3|  50|
|8|	-30|	-10|	30|	0|	130|	138|	40|	19.3|100|
|9|	-30	|-10	|30	|0.33	|130	|138	|40	|18.4|50|
|10|	-30|	-10|	30|	0.33|	130|	138|	40|	18.3|100|
|11|-30	|-10	|30	|0.5	|130	|138	|40	|18.6| 50|
|12|	-30|	-10|	30|	0.5|	130|	138|	40|	18.6| 100|
|13|	-30	|-10	|30	|-0.2	|130	|138	|40	|19.5|50|
|14|	-30|	-10|	30|	-0.2|	130|	138|	40|	19.5|100|
|15|5	|130|	30|	0.2	|120|	40|	10	|13.9|  50|
|16	|5	|130	|30	|0.2	|120	|40	|10	|14|100|
|17|5	|130	|30|	-0.33|	120|	40	|10	|12.9| 50| 
|18|	5	|130|	30|	-0.33|	120|	40|	10|	12.9|100|
|19|5	|130	|30	|-0.45	|120	|40	|10	|13.4|  50|
|20	|5	|130	|30	|-0.45	|120	|40	|10	|13.4|   100|

For each planning time and steps acado code was generated.

#### Generating ACADO code
* Install [ACADO Toolkit](https://acado.github.io/install_linux.html)
* Copy ```getting_started.cpp``` from ```acadoOptim``` folder and paste it to ```/ACADOtoolkit/examples/code_generation/mpc_mhe```.
* Edit ```getting_started.cpp``` for desired settings (steps, planning time).
* Navigate to ```ACADOtoolkit/build``` and run ```make code_generation_getting_started```.
* Finally run ```./code_generation_getting_started``` in folder ```/ACADOtoolkit/examples/code_generation/mpc_mhe```. 








