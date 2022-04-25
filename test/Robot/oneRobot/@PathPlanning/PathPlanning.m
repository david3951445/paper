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

            %% show result path
            % show(map);
            % hold on;
            % plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-', 'DisplayName','tree expansion'); % tree expansion
            % plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2, 'DisplayName','path') % draw path
            % legend

            %% Interpolate path, make it more smooth and increase density
            r1 = pthObj.States(:, 1:2)';
            len1 = length(r1);
            t1 = linspace(0,1,len1);

            len2 = (len1-1)*pp.INTERP_DENSITY(1)+1;
            t2 = linspace(0,1,len2);
            r2 = zeros(2, len2);
            r2 = [
                interp1(t1, r1(1, :), t2);
                interp1(t1, r1(2, :), t2);
            ];

            fx = fit(t2', r2(1,:)', 'SmoothingSpline');
            fy = fit(t2', r2(2,:)', 'SmoothingSpline');
            len3 = len2*pp.INTERP_DENSITY(2);
            t3 = linspace(0,1,len3);
            r3 = [
                feval(fx, t3)';
                feval(fy, t3)';
            ];

            pp.start = start;
            pp.goal = goal;
            pp.r = r3;
            pp.tree = solnInfo.TreeData;
        end 

        Save(pp, filename, whichVar) % Save property
        pp = Load(pp, filename) % Load property
    end
end

