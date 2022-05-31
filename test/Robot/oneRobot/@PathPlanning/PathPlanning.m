classdef PathPlanning
    %Path Planning Class 
    
    properties (Constant)
        INTERP_DENSITY = [2 3] % interp1, smoothspline
        RESOLUTION = 100 % cells per meter.
        MAX_CON_DIS = .5 % Max Connection Distance of RRT
        PATH = ['data/' mfilename]
    end

    properties
        r % path
        tree % span tree of RRT
        map % occupancy Map
        start % start point
        goal % goal point
    end
    
    methods
        function pp = PathPlanning()
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
            start = [map.XWorldLimits(1) map.YWorldLimits(1) 0];
            goal = [map.XWorldLimits(2) map.YWorldLimits(2) 0];
            rng(1); % repeatable result
            [pthObj, solnInfo] = planner.plan(start,goal);
            r3 = post_processing(pthObj.States(:, 1:2)', pp.INTERP_DENSITY) % Interpolate path, make it more smooth and increase density

            pp.start = start;
            pp.goal = goal;
            pp.r = r3;
            pp.map = map;
            pp.tree = solnInfo.TreeData;
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

        r3 = post_processing(r1, INTERP_DENSITY) % post processing of ref traj after planning
        Save(pp, filename, whichVar) % Save property
        pp = Load(pp, filename) % Load property
    end
end

