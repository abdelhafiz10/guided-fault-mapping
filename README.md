# Guided Fault Mapping using Topographic Metrics
## Maddie Schwarz, Abdel Hafiz, Mindy Zuckerman, Aswayuja Koduri

## How to run this script?
- Open this [notebook](https://github.com/abdelhafiz10/guided-fault-mapping/blob/main/Stage%201%20DEM%20Pre-processing%20Script%20--%20Guided%20Fault%20Scarp%20Mapping.ipynb) first to extract fault scarp area 
- Then open this [notebook](https://github.com/abdelhafiz10/guided-fault-mapping/blob/main/Stage%202%20Generate%20Waypoints%20--%20Guided%20Fault%20Scarp%20mapping.ipynb) after you got scarp polygon from previous notebook
- Then use the [waypoints.csv](https://github.com/abdelhafiz10/guided-fault-mapping/blob/main/waypoints.csv) that you got from previous notebook as input for [final_v5.py](https://github.com/abdelhafiz10/guided-fault-mapping/blob/main/final_v5.py), make sure the location of [waypoints.csv](https://github.com/abdelhafiz10/guided-fault-mapping/blob/main/waypoints.csv) in [final_v5.py](https://github.com/abdelhafiz10/guided-fault-mapping/blob/main/final_v5.py) is correct
- Run 'roslaunch cps_challenge_2020 phase-1.launch' in terminal
- Open QGroundControl
- Run 'python 3 /your_script_folder/final_v5.py'
- Run 'roslaunch rtabmap_ros rtabmap.launch' in terminal

## Simulation Video :

- [Simulation Video](https://youtu.be/3vuvNAtDelU)

