Contain all scripts(function, class, main script)

- functions
    - External : External toolbox.
        - to solve LMI
            - sdpt3 : linear programing solver
            - YALMIP :
            Modeling language or High level language for optimizes problem 
                - Web page : https://yalmip.github.io
        - to linearlize system
            - tptool : tensoe product tool

    - Matrix : About matrix calculation

    - Hinf_Therom : Solving control or observer gain with H infinity performance

    - Linearization : Methods for linearize a nonlinear system
        - Fuzzy : T-S fuzzy approach
        - TPmodel : Tensor product transmormation approach
        - checkLinearizedSum.m : Check the sum of interpolation function of local linear systems

    - System : Some time systems with state-space representation
        - Agent : agent model
        - UAV : UAV model
        - Robot : Biped robot model
        - LPV : Linear Parameter Varing system

- URTS main script :
    - Coppelia simulation : UAV and robot simulation. Using MATLAB to remote control Coppelia
    - Robot : one robot, smoothed model
    - UAV : one UAV, smoothed model