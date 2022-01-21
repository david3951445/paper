clc;clear;
%%%  dxdt = J(x)v %%% 
% 
% syms phi0(t) phi1(t) t d
% 
% phi0 = symfun(phi0(t), t);
% phi1 = symfun(phi1(t), t);
% 
% J = [cos(phi1)*cos(phi0-phi1) 0;
%      sin(phi1)*cos(phi0-phi1) 0;
%            1/d*sin(phi0-phi1) 0;
%                             0 1];
%      
% dJdt = diff(J,t); 
syms phi0 phi1 d dphi0dt dphi1dt
dJtdt = [- sin(phi1)*cos(phi0 - phi1)*dphi1dt - cos(phi1)*sin(phi0 - phi1)*(dphi0dt - dphi1dt), 0;
           cos(phi1)*cos(phi0 - phi1)*dphi1dt - sin(phi1)*sin(phi0 - phi1)*(dphi0dt - dphi1dt), 0;
                                                      (cos(phi0 - phi1)*(dphi0dt - dphi1dt))/d, 0;
                                                                                             0, 0];
save dJtdt.mat
                                          
