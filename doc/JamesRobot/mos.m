%% 
%案例一:求 x^2+y^2在x,y∈[-2,2]上的最小值
clc;clear;
%生成2*1的矩阵变量
x = sdpvar(2,1);
%限制条件
F = [-2 <= x <= 2];
%目标函数
obj = x(1)^2+x(2)^2;
%求解
optimize(F,obj);
%取得值以及对应的x的值
optobj = value(obj)
optx = value(x)

%% 
% 案例二:指派问题,A,B,C为3个员工,P,Q,R为3样工作,成本表如下:
% 	A	B	C
% P	9	6	2
% Q	3	1	4
% R	5	6	10
clc;clear;
%设x(a,b)==1为a做b工作
x=binvar(3,3);
%成本矩阵
work=[9,6,2;3,1,4;5,6,10];
%限制条件,每人只能做一份工作
F=[sum(x)==1];
%目标函数,总花费最小
obj=sum(sum(x.*work));
%启动求解器
optimize(F,obj);
%获取结果
optx=value(x)
optObj=value(obj)

%% 
% YALMIP让我们用MATLAB语言方便地调用专业的求解器,而借助于MATLAB自带的一些函数,我
% 们可以更容易的解决各种规划问题.

% 但有一点需要注意,MATLAB的语法虽好用,但也不是所有函数都可以与YALMIP结合,例如神
% 经网络,模拟退火的函数是不可以当做限制函数或限制条件参与规划的.