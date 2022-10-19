This folder is the simulation of a biped-robot. For more detail, please check out my paper in the folder "..\..\..\paper"

There are three main script:
1. oneRobot_FaFs.m
    - Construct system matrices
    - Calculate gains using LMI solver
    - Calculate paths using path planning algorithm
    - Calculate trajectories 
    - Plot the results. 5 figures are made:
        - path planning result
        - local motion planning result
        - tracking control results
            - states, estimated states, reference states
            - estimation of acuator and sensor faults
            - control effort
2. robot_simulink.m
    Use the physical robot created by simulink to show the results of the joint space state trajectory from oneRobot_FaFs.m
3. RobotSimulation.m
    Use the robot footprint to show the results of the task space state trajectory from oneRobot_FaFs.m
