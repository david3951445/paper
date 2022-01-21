clc;clear;
syms phi
T = [...
     cos(phi),sin(phi),0,0,0;
    -sin(phi),cos(phi),0,0,0;
            0,       0,1,0,0;
            0,       0,0,1,0;
            0,       0,0,0,1
    ];


invT = simplify(inv(T));