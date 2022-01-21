clc;clear;
%%%  dxdt = J(x)v %%% 

syms phi0 phi1 d
assume(phi0,'real');
assume(phi1,'real');
assume(d,'real');

Jt = [cos(phi1)*cos(phi0-phi1) 0;
      sin(phi1)*cos(phi0-phi1) 0;
            1/d*sin(phi0-phi1) 0;
                             0 1];
pinvJt = simplify(pinv(Jt));       

save Jt.mat

