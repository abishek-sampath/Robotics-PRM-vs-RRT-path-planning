# Robotics-PRM-vs-RRT-path-planning
Project work done for Robotics course at Northeastern. Comparison of path planning algorithms - PRM vs RRT, in an environment with 5 spherical obstacles

## Steps to run the program
 - Clone/Download the repo to your local environment.
 - Matlab and Peter Corke’s Robotics toolbox should be installed.
 - Open Matlab with the project folder.
 - Run *𝑠𝑡𝑎𝑟𝑡𝑢𝑝_𝑟𝑣𝑐* in the command window to startup the Robotics Toolbox
 - Run the program with the following command syntax:
 

>     start_path_planning(algorithmType,startX, startY, startZ, endX, endY, endZ)
>       Required:
>             algorithmType
>                 ('PRM' or 'RRT')
>       Optional:
>             startX, startY, startZ, endX, endY, endZ
>                 (Default coordinates will be used)
