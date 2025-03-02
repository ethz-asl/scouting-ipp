## Helper Scripts
This repository contains a set of helpful scripts to streamline the workflow.
```bash
roscd ros_planner_planexp
cd ../helper_scripts/
```
|Name|Description|
|--|--|
| analyse_map.py | The analyse_map script creates a report containing best path estimates, cost distributions, optimal path cost and visualizations of the map. |
| check_map.py | The check_map script provides statistics on cost distributions and allows to inspect pixels by coordinate (px/m). |
| compute_baseline_performance.py | The compute_baseline_performance script computes the baseline performance metrics from a given data file in csv format. |
| create_config_space_map.py | The create_config_space_map script creates planner maps based on cost and obstacle maps and allows additional adjustments (e.g. obstacle inflation, blurring, ...) .  |
| create_map_config.py | The create_map_config script creates all necessary configuration files to run planner experiments on the provided map. |
| create_maps.py | The create_maps script is a dirty implementation of a script creating multiple scaled versions of a set of base maps with multiple different obstacle distributions. |
| create_map_visualization.py | The create_map_visualization script creates a map visualization file from a given map file. |
| create_path_animation.py | The create_path_animation script creates a visualization of the rover path evolution in gif format . |
| extract_obstacles.py | The extract_obstacles script is a version of the create_config_space_map script utilizing a different erosion and dilation function. |
| label_replacer.py | The label_replacer script is a tool to change labels in a map file either by direct map or by command line UI. |
| map_conversion_functions.py | The map_conversion_functions file contains the functions to convert between color and label space  introduced by [OAISYS](https://doi.org/10.1109/IROS51168.2021.9636644). |
| oaisys_image_gen.py | The oaisys_image_gen script is a dirty implementation of a script creating maps using OAISYS. |
| report_functions.py | The report_functions file contains functions used to create a map analysis report. |
| visualization_dirty.py | The visualization_dirty script creates a visualization (gif) of the rover path estimate, the scout path and the currently explored area. |
| visualize_rrt_star_evaluator_path.py | The visualize_rrt_star_evaluator_path script creates a visualization of the rover path estimate evolution in gif format. |

**Note:** Most scripts contain one main function that can be used to integrate the functionality in a new function or script.

### analyse_map.py
The analyse_map script creates a report containing best path estimates, cost distributions, optimal path cost and visualizations of the map. Computer readable results can be created by using the related command line arguments.
```
python analyse_map.py [-h] [--dest DESTINATION_FOLDER] [-m MAP_CONFIG_FILE] [-n NUM_CLASSES] [-i ITERATIONS] [-s] [-p] [-pv] [-o] source

Analyses given map.

positional arguments:
  source                input file (map) or folder

optional arguments:
  -h, --help            show this help message and exit
  --dest DESTINATION_FOLDER
                        output folder where evaluation data shall be stored
  -m MAP_CONFIG_FILE    map_config_file of the stored map
  -n NUM_CLASSES        number of data classes used for segmentation
  -i ITERATIONS         number of iterations to find best cost
  -s                    generate the statistics file
  -p                    generate the path file
  -pv                   generate the path visualization file
  -o                    override all files
```

### check_map.py
The check_map script provides statistics on cost distributions and allows to inspect pixels by coordinate (px/m).
```
python check_map.py [-h] [-m MAP_CONFIG_FILE] map

Allows inspection of map at locations and map statistics.

positional arguments:
  map                 input file (map) or folder

optional arguments:
  -h, --help          show this help message and exit
  -m MAP_CONFIG_FILE  map_config_file of the stored map
```

### compute_baseline_performance.py
The compute_baseline_performance script computes the baseline performance metrics from a given data file in csv format.
```  
python compute_baseline_performance.py [-h] [-c MAP_CONFIG_FILE] [-m MAP] [-d DESTINATION] [-r] data

Computes performance metrics from given baseline data files.

positional arguments:
  data                  input file (csv) or folder

optional arguments:
  -h, --help            show this help message and exit
  -c MAP_CONFIG_FILE    map_config_file of the stored map
  -m MAP                Path to the map file
  -d DESTINATION        Destination where to store results
  -r, --run_comparison  Do a comparison in the known evaluation structure.
```

### create_config_space_map.py
The create_config_space_map script creates planner maps based on cost and obstacle maps and allows additional adjustments (e.g. obstacle inflation, blurring, ...) .
```  
create_config_space_map.py [-h] [-s SOURCE] [--dest DESTINATION_FOLDER] [-n NUM_CLASSES] [-o]
                                  [--ids [OBSTACLE_IDS ...]] [-b] [--start START START]
                                  [--goal GOAL GOAL] [-w SQUARE_WIDTH] [-d DILATE] [-e ERODE] [-ir]
                                  map

Extracts obstacles from maps and overlays them on other maps.

positional arguments:
  map                   input map file

optional arguments:
  -h, --help            show this help message and exit
  -s SOURCE             obstacle file or folder
  --dest DESTINATION_FOLDER
                        output folder where maps shall be stored
  -n NUM_CLASSES        number of data classes used for segmentation
  -o                    override all files
  --ids [OBSTACLE_IDS ...]
                        IDs to be considered obstacles
  -b                    apply blurring before adding the obstacles
  --start START START   start pixel
  --goal GOAL GOAL      goal pixel
  -w SQUARE_WIDTH       width of the area arround start and goal to keep free of obstacles, if not
                        defined nothing is kept free
  -d DILATE             dilate obstacles n times
  -e ERODE              erode obstacles n times
  -ir                   inflate obstacles by the half size of the Perseverance rover
```

### create_map_config.py
The create_map_config script creates all necessary configuration files to run planner experiments on the provided map.
```  
create_map_config.py [-h] [--dest DESTINATION_FOLDER] [-n NUM_CLASSES] [-sp] [-o]
                            [--oid [OBSTACLE_IDS ...]] [-c]
                            source

Creates configuration file for given map.

positional arguments:
  source                input file (map) or folder

optional arguments:
  -h, --help            show this help message and exit
  --dest DESTINATION_FOLDER
                        output folder where data shall be stored
  -n NUM_CLASSES        number of data classes used for segmentation
  -sp                   supress generation of planner config file
  -o                    override all files
  --oid [OBSTACLE_IDS ...]
                        IDs to be considered obstacles
  -c                    compute optimal path cost
```

### create_maps.py
The create_maps script is a dirty implementation of a script creating multiple scaled versions of a set of base maps with multiple different obstacle distributions. Parameters have to be changed directly in the script.
``` 
python create_maps.py
```

### create_map_visualization.py
The create_map_visualization script creates a map visualization file from a given map file. The range file is a yaml file containing a min and a max value for the color scale.
```  
create_map_visualization.py [-h] [--dest DESTINATION_FOLDER]
                                   [-n NUM_CLASSES] [-o] [-u UNKNOWN_ID]
                                   [--oid [OBSTACLE_IDS ...]] [-i]
                                   [--range RANGE_FILE]
                                   source

Creates visualization file for given map.

positional arguments:
  source                input file (map) or folder

optional arguments:
  -h, --help            show this help message and exit
  --dest DESTINATION_FOLDER
                        output folder where data shall be stored
  -n NUM_CLASSES        number of data classes used for segmentation
  -o                    override all files
  -u UNKNOWN_ID         label of unknown area
  --oid [OBSTACLE_IDS ...]
                        IDs to be considered obstacles
  -i                    ignore range file (defined and default)
  --range RANGE_FILE    range file containing the ranges used for
                        visualization

```

### create_path_animation.py
The create_path_animation script creates a visualization of the rover path evolution in gif format .
``` 
create_path_animation.py [-h] [--dest DESTINATION_FOLDER] [-m MAP_FILE] [-n N_RUN] source

Visualizes rover path evolution as gif.

positional arguments:
  source                input file (pickle)

optional arguments:
  -h, --help            show this help message and exit
  --dest DESTINATION_FOLDER
                        output folder where evaluation data shall be stored
  -m MAP_FILE           path to the map in which the paths should be plotted
  -n N_RUN              number of the run to be plotted 
```

### extract_obstacles.py
The extract_obstacles script is a version of the create_config_space_map script utilizing a different erosion and dilation function. The use of the create_config_space_map script is recommended.
``` 
extract_obstacles.py [-h] [-s SOURCE] [--dest DESTINATION_FOLDER]
                            [-n NUM_CLASSES] [-o] [--ids [OBSTACLE_IDS ...]]
                            [-b] [--start START START] [--goal GOAL GOAL]
                            [-w SQUARE_WIDTH] [-d DILATE] [-e ERODE] [-ir]
                            map

Extracts obstacles from maps and overlays them on other maps.

positional arguments:
  map                   input map file

optional arguments:
  -h, --help            show this help message and exit
  -s SOURCE             obstacle file or folder
  --dest DESTINATION_FOLDER
                        output folder where maps shall be stored
  -n NUM_CLASSES        number of data classes used for segmentation
  -o                    override all files
  --ids [OBSTACLE_IDS ...]
                        IDs to be considered obstacles
  -b                    apply blurring before adding the obstacles
  --start START START   start pixel
  --goal GOAL GOAL      goal pixel
  -w SQUARE_WIDTH       width of the area arround start and goal to keep free
                        of obstacles, if not defined nothing is kept free
  -d DILATE             dilate obstacles n times
  -e ERODE              erode obstacles n times
  -ir                   inflate obstacles by the half size of the Perseverance

```

### label_replacer.py
The label_replacer script is a tool to change labels in a map file either by direct map or by command line UI.
```
label_replacer.py [-h] [--dest DESTINATION_FOLDER] [-o] [-n NUM_CLASSES] [--map MAP_YAML] [-s] [-pf POST_FIX] source

Replaces labels in a given map file.

positional arguments:
  source                input file (map) or folder

optional arguments:
  -h, --help            show this help message and exit
  --dest DESTINATION_FOLDER
                        output folder where new maps shall be stored
  -o                    override all files
  -n NUM_CLASSES        number of data classes used for segmentation
  --map MAP_YAML        yaml file used to directly map labels to new ones
  -s                    supress post-fix for output file
  -pf POST_FIX          alternative post-fix for file
```

### map_conversion_functions.py
The map_conversion_functions file contains the functions to convert between color and label space  introduced by [OAISYS paper](https://doi.org/10.1109/IROS51168.2021.9636644).

### oaisys_image_gen.py
The oaisys_image_gen script is a dirty implementation of a script creating maps using OAISYS. Parameters have to be changed directly in the script.
```bash  
python oaisys_image_gen.py  
```

### report_functions.py
The report_functions file contains functions used to create a map analysis report.

### visualization_dirty.py

The visualization_dirty script creates a visualization (gif) of the rover path estimate, the scout path and the currently explored area. Parameters and paths have to be changed directly in the script. An example is the gif shown at the top.

```  
python visualization_dirty.py
```

### visualize_rrt_star_evaluator_path.py
The visualize_rrt_star_evaluator_path script creates a visualization of the rover path estimate evolution in gif format.
``` 
visualize_rrt_star_evaluator_path.py [-h] [--dest DESTINATION_FOLDER] [--size MAP_SIZE MAP_SIZE] [--map MAP_PATH] data_folder

Visualizes the rover path estimate evolution.

positional arguments:
  data_folder           output folder of the logged data

optional arguments:
  -h, --help            show this help message and exit
  --dest DESTINATION_FOLDER
                        output folder where plots shall be stored
  --size MAP_SIZE MAP_SIZE
                        size of map in meters [width x height]
  --map MAP_PATH        path of map visualization file
```