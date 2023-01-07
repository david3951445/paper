%An example of eliminating the non-smooth properties and outliers that result from numerical differentiation of a signal

clc; clear; close all;
qr = load('../src/URTS main script/Robot/data/Robot.mat').qr;

DIM_F   = size(qr, 1);
dt      = 0.001;
Fs      = 1/dt;
t       = linspace(0,1,size(qr, 2));

for i = 1 : DIM_F
r = qr(i,:);

%% First difference
% Origin
dr = diff(r, 1, 2)/dt;
% Smoothed
dr_Smoothed = filloutliers(dr,'clip','movmedian', 10, 'ThresholdFactor', 1); % numercial differenciation will produce outliers 
dr_Smoothed = smoothdata(dr_Smoothed);

%% Second difference
% Origin
ddr = diff(dr_Smoothed, 1, 2)/dt;
% Smoothed
ddr_Smoothed = smoothdata(ddr);

%% Resize
dr = AddZerosToFrontOfArray(dr, 1);
ddr = AddZerosToFrontOfArray(ddr, 2);
dr_Smoothed = AddZerosToFrontOfArray(dr_Smoothed, 1);
ddr_Smoothed = AddZerosToFrontOfArray(ddr_Smoothed, 2);

%% Store results
tr.r(i,:) = r;
tr.dr(i,:) = dr;
tr.ddr(i,:) = ddr;

tr.dr_Smoothed(i,:) = dr_Smoothed;
tr.ddr_Smoothed(i,:) = ddr_Smoothed;
end

%% Plot
figure
plot(t, tr.r)
title('r(t)')
figure
plot(t, tr.dr)
title('r''(t)')
figure
plot(t, tr.dr_Smoothed)
title('r''(t), smoothed')
figure
plot(t, tr.ddr)
title('r''''(t)')
figure
plot(t, tr.ddr_Smoothed)
title('r''''(t), smoothed')

%% Functions
function arr = AddZerosToFrontOfArray(arr, num) % diff() will result missing data, resize it
    arr = [zeros(1, num) arr];
end
