clc;clear;close all
load 'com0_trajectory_reference_model.mat'
%%

%%% when time period of one gait is 2*T %%%
%%% 0 ~ 2T is matching to pi ~ -pi %%% 
Tp = 4; %% time period of a single step
tp = 0:hr:2*Tp; %% time instant of a complete step (2 single step)
len = length(tp); 

step = t11/(2*Tp);
hf = 0.01; %% the height of gait
c = 0.5; %% the constant multiply to v --> deteremine the gait length
dt = hr; 

for i = 1:length(v)
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = (floor(i/len)-(floor(i/len)-1))*len;
    end
    gait(:,i) = leg_gait_s(hf,c,len,v(i),index,dt);
    disp(i);
end


save leg_trajectory_test_v7.mat
