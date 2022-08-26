% An example of eliminating the non-smooth properties and outliers that result from numerical differentiation of a signal
clc; clear; close all;
qr = load('../data/qr.mat').qr;

DIM_F   = size(qr, 1);
dt      = 0.001;
Fs      = 1/dt;
t       = linspace(0,1,size(qr, 2));

for i = 1 : DIM_F
r = qr(i,:);

dr = diff(r, 1, 2)/dt;
dr = filloutliers(dr,'clip','movmedian', 10, 'ThresholdFactor', 1); % numercial differenciation will produce outliers 
dr = smoothdata(dr);

ddr = diff(dr, 1, 2)/dt;    
ddr = smoothdata(ddr);

dr = [zeros(1, 1) dr]; % diff() will result in 1 missing data, resize it
ddr = [zeros(1, 2) ddr]; % diff() will result in 1 missing data, resize it

tr.r(i,:) = r;
tr.dr(i,:) = dr;
tr.ddr(i,:) = ddr;
end

figure
plot(t, tr.r)
figure
plot(t, tr.dr)
figure
plot(t, tr.ddr)