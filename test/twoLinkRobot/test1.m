% tpmodel but just calculate A(2,:) and A(4,:)
clc; clear;
addpath(genpath('..\..\src'))
addpath(genpath('function'))
rb = Robot();
rb.Al.val = {
    @(p)0               @(p)1               @(p)0               @(p)0
    @(p)rb.setAl(p, 21)  @(p)rb.setAl(p, 22)  @(p)rb.setAl(p, 23)  @(p)rb.setAl(p, 24)
    @(p)0               @(p)0               @(p)0               @(p)1
    @(p)rb.setAl(p, 41)  @(p)rb.setAl(p, 42)  @(p)rb.setAl(p, 43)  @(p)rb.setAl(p, 44)
};
rb.Al.domain = 2*[-1 1; -1 1; -1 1; -1 1];
rb.Al.gridsize = 1*[10 10 10 10];
rb.Al.SV_TOLERANCE = 0.001;
rb.Al.num_p = length(rb.Al.gridsize); % length of parameter vector of lpv system (p = [x1, x2, x3, x4])
rb.Al.dep = zeros([size(rb.Al.val) rb.Al.num_p]);
rb.Al.dep(2,1,:) = [1 0 1 0];
rb.Al.dep(2,2,:) = [1 1 1 0];
rb.Al.dep(2,3,:) = [1 0 1 0];
rb.Al.dep(2,4,:) = [1 0 1 1];
rb.Al.dep(4,1,:) = [1 0 1 0];
rb.Al.dep(4,2,:) = [1 1 1 0];
rb.Al.dep(4,3,:) = [1 0 1 0];
rb.Al.dep(4,4,:) = [1 0 1 1];
rb.A = TPmodel(rb.Al);

rmpath(genpath('function'))
rmpath(genpath('..\..\src'))