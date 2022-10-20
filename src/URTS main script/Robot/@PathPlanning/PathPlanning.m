classdef PathPlanning
    %Path Planning Class 
    
    properties (Constant)
        INTERP_DENSITY = [2 3] % interp1, smoothspline
        RESOLUTION = 100 % cells per meter.
        MAX_CON_DIS = .5 % Max Connection Distance of RRT
        PATH = ['data/' mfilename]
    end

    properties
        sigma % path
        r % smoothed path
        tree % span tree of RRT
        map % occupancy Map
        start % start point
        goal % goal point
    end
    
    methods
        function pp = PathPlanning(start, goal)
            %% Construct map
            X = imread('map.png');
            imageCropped = X(:,:,1);
            imageNorm = double(imageCropped)/255;
            imageOccupancy = 1 - imageNorm;
            map = occupancyMap(imageOccupancy, pp.RESOLUTION);

            %% Find path
            ss = stateSpaceDubins;
            ss.MinTurningRadius = 0.2;
            ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

            sv = validatorOccupancyMap(ss);
            sv.Map = map;
            sv.ValidationDistance = 0.01;

            planner = plannerRRTStar(ss,sv);
            planner.MaxConnectionDistance = pp.MAX_CON_DIS;
            if nargin == 0
                start = [map.XWorldLimits(1) map.YWorldLimits(1) 0];
                goal = [map.XWorldLimits(2) map.YWorldLimits(2) 0];
            end
            rng(2); % repeatable result
            [pthObj, solnInfo] = planner.plan(start, goal);

            pp.map = map;
            pp.sigma = pthObj.States(:, 1:2)';
            pp.r = fit_linear_spline(pp.sigma, pp.INTERP_DENSITY); % Interpolate path, make it more smooth and increase density
            pp.tree = solnInfo.TreeData;
            pp.start = start;
            pp.goal = goal;
        end 

        function Plot(pp)
            %% show result path
            fig = figure;
            show(pp.map);
            hold on;
            plot(pp.tree(:,1), pp.tree(:,2),'.-', 'DisplayName','tree expansion'); % tree expansion
            % plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2, 'DisplayName','path') % draw path
            legend
            FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
            saveas(fig, FILE_NAME)
        end

        r3 = Post_processing(pp, r1) % post processing of ref traj after planning
        Save(pp, filename, whichVar) % Save property
        pp = Load(pp, filename) % Load property
    end
end

