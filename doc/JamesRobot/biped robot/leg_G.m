clc;clear;close all
syms th dthdt
assume(th,'real');
G = [...
    cos(th),sin(th) ,0,0,0,0,0,0;
    sin(th),-cos(th),0,0,0,0,0,0;
    0,0,0,1,0,0,0,0
    ];
G_pinv = simplify(pinv(G));

dpinvGdt=[...
    -dthdt*sin(th),dthdt*cos(th),0;
    dthdt*cos(th) ,dthdt*sin(th),0;
    0,0,0;
    0,0,0;
    0,0,0;
    0,0,0;
    0,0,0;
    0,0,0
    ];


save 'G.mat'