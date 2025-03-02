# Traversing Mars: Cooperative Informative Path Planning to Efficiently Navigate Unknown Scenes
![](https://github.com/ethz-asl/scouting-ipp/assets/36043993/f0d283df-9de1-4eb7-b4b6-66a82a389194)


This repository will contain the code for our informative path planning (IPP) framework allowing a mobile scout (e.g. an MAV) to explore the cost-optimal path of a follower robot (e.g. a rover) to a goal location in unknown environments. (Models: Courtesy NASA/JPL-Caltech.)

> This work was supported by ETH Research Grant AvalMapper ETH-10 20-1 and the Swiss National Science Foundation (SNSF) grant No. 214489.


## Credits
If you find this useful for your research, please consider citing our paper:

* F. M. Rockenbauer, J. Lim, M. G. Müller, R. Siegwart and L. Schmid, "**Traversing Mars: Cooperative Informative Path Planning to Efficiently Navigate Unknown Scenes**" in *IEEE Robotics and Automation Letters*, vol. 10, no. 2, pp. 1776-1783, Feb. 2025, doi: 10.1109/LRA.2024.3513036 [ [IEEE](https://doi.org/10.1109/LRA.2024.3513036) ] [ [ArXiv](https://arxiv.org/abs/2406.05313) ]
```bibtex
  @ARTICLE{10783051,
    author={Rockenbauer, Friedrich M. and Lim, Jaeyoung and Müller, Marcus G. and Siegwart, Roland and Schmid, Lukas},
    journal={IEEE Robotics and Automation Letters}, 
    title={Traversing Mars: Cooperative Informative Path Planning to Efficiently Navigate Unknown Scenes}, 
    year={2025},
    volume={10},
    number={2},
    pages={1776-1783},
    keywords={Robots;Costs;Space exploration;Path planning;Navigation;Planning;Mobile robots;Green products;Wheels;Uncertainty;View planning for SLAM;path planning for multiple mobile robots or agents;space robotics and automation},
    doi={10.1109/LRA.2024.3513036}
    }
  ```

## Installation
There are two ways to install the planner. The easier way using a docker container is described below. If you want to install the planner into your local ros workspace, follow the istructions [here](https://github.com/ethz-asl/scouting-ipp/blob/main/install_local.md). 

1. Setup docker as described [here](https://docs.docker.com/engine/install/)

2. Clone the repository: 
```shell script 
git clone https://github.com/ethz-asl/scouting-ipp.git
git clone git@github.com:ethz-asl/scouting-ipp.git
``` 

3. Build the docker image: 
```shell script
cd scouting-ipp/docker/ 
docker build -t scouting_ipp:1.0 -f scouting_ipp.Dockerfile .
```

4. Set up the data folder: 
```shell script 
cd ../..
cp scouting-ipp/docker/data <your_data_directory>
``` 
**Note:** The data directory will become multiple GBs very quickly.

5. Run a docker container and mount the data directory into it: 
```shell script 
docker run -it --mount src=<your_data_directory>,target=/home/root/data,type=bind scouting_ipp:1.0 /bin/bash
```

6. Run the planner either using the `examle.launch` file:
```shell script 
roslaunch ros_planner_planexp example.launch 
```
or run an experiment: 
```shell script 
roscd ros_planner_planexp
source scripts/run_experiment.sh
```

## Animation 
![](https://user-images.githubusercontent.com/34751524/216842104-2a24d9e7-2961-44d9-bccd-ed279f97582e.gif)

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License
[MIT](https://choosealicense.com/licenses/mit/)
