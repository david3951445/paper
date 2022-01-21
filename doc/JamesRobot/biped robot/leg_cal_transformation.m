clc;clear;
syms c1 s1 c2 s2 c3 s3 c4 s4 c5 s5 c6 s6 L1 L2 L3 L4 L5 L6
syms x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4 x5 y5 z5 x6 y6 z6 % com
%%% DH model %%%
A4 = [c4 -s4 0 L5*c4;
      s4  c4 0 L5*s4;
       0   0 1 0;
       0   0 0 1];
A3 = [c3 -s3 0 L4*c3;
      s3  c3 0 L4*s3;
       0   0 1     0;
       0   0 0     1];
A2 = [ s2   0  c2  0;
      -c2   0  s2  0;
        0  -1   0  0;
        0   0   0  1];
A1 = [c1 0  s1   0;
      s1 0 -c1   0;
       0 1   0 -L3;
       0 0   0   1];
A1_inv = [c1  s1  0   0;
           0   0  1  L3;
          s1 -c1  0   0;
           0   0  0   1];
A2_inv = [s2 -c2  0  0;
           0   0 -1  0;
          c2  s2  0  0;
           0   0  0  1];  
% used for kinematics 
A = A1*(A2*(A3*A4));
AA = A2*(A3*A4);
AAA = A3*A4;
B = A2_inv*A1_inv;

% % used for dynamics 
% A = A1*(A2*(A3*(A4*(A5*A6))));

%%% tool and base transformation %%%
d_tool = [-90 0 90]; %zyx
r_tool = deg2rad(d_tool);
rotm_tool = eul2rotm(r_tool);
d_base = [90 0 0]; %zyx
r_base = deg2rad(d_base);
rotm_base = eul2rotm(r_base);
% output small value to 0 
for i = 1:3
    for j = 1:3
        if abs(rotm_tool(i,j)) < 1e-6
            rotm_tool(i,j) = 0;
        end
        if abs(rotm_base(i,j)) < 1e-6
            rotm_base(i,j) = 0;
        end
    end
end

% used for kinematics 
%%% right leg %%%
A_base_r = cat(2,cat(1,rotm_base,zeros(1,3)),[0;-L1;-L2;1]); % frame{b} to frame{0}
A_tool_r = cat(2,cat(1,rotm_tool,zeros(1,3)),[0;0;0;1]); % frame{4} to frame{tool}
Tr = A_base_r*A*A_tool_r; % from frame{b} to frame{tool}
T0r = A_base_r; % frame{0} is respect to frame{b} --> motor 1
T1r = A_base_r*A1; % frame{1} is respect to frame{b} --> motor 2
T2r = A_base_r*A1*A2; % frame{2} is respect to frame{b} --> motor 3
T3r = A_base_r*A1*A2*A3; % frame{3} is respect to frame{b} --> motor 4
T4r = A_base_r*A;

%%% left leg %%% 
A_base_l = cat(2,cat(1,rotm_base,zeros(1,3)),[0;L1;-L2;1]); % frame{b} to frame{0}
A_tool_l = cat(2,cat(1,rotm_tool,zeros(1,3)),[0;0;0;1]); % frame{4} to frame{tool}
Tl = A_base_l*A*A_tool_l; % from frame{b} to frame{tool}
T0l = A_base_l; % frame{0} is respect to frame{b} --> motor 1
T1l = A_base_l*A1; % frame{1} is respect to frame{b} --> motor 2
T2l = A_base_l*A1*A2; % frame{2} is respect to frame{b} --> motor 3
T3l = A_base_l*A1*A2*A3; % frame{3} is respect to frame{b} --> motor 4
T4l = A_base_l*A;


% % used for dynamics 
% syms th1 th2 th3 th4 th5 th6 L1 L2 L3 L4 L5 L6
% % used for dynamics 
% A6 = [c6 -s6 0 L6*c6;
%       s6  c6 0 L6*s6;
%        0   0 1     0;
%        0   0 0     1];
% A5 = [c5  0 -s5 0;
%       s5  0  c5 0;
%        0 -1   0 0;
%        0  0   0 1];
% A4 = [c4 0  s4 L5*c4;
%       s4 0 -c4 L5*s4;
%        0 1   0     0;
%        0 0   0     1];
% A3 = [c3 -s3 0 L4*c3;
%       s3  c3 0 L4*s3;
%        0   0 1     0;
%        0   0 0     1];
% A2 = [ s2   0  c2  0;
%       -c2   0  s2  0;
%         0  -1   0  0;
%         0   0   0  1];
% A1 = [c1 0  s1   0;
%       s1 0 -c1   0;
%        0 1   0 -L3;
%        0 0   0   1];
% % used for dynamics 
% A = A1*(A2*(A3*(A4*(A5*A6))));
% 
% %%% tool and base transformation %%%
% d_tool = [-90 0 90]; %zyx
% r_tool = deg2rad(d_tool);
% rotm_tool = eul2rotm(r_tool);
% d_base = [90 0 0]; %zyx
% r_base = deg2rad(d_base);
% rotm_base = eul2rotm(r_base);
% % output small value to 0 
% for i = 1:3
%     for j = 1:3
%         if abs(rotm_tool(i,j)) < 1e-6
%             rotm_tool(i,j) = 0;
%         end
%         if abs(rotm_base(i,j)) < 1e-6
%             rotm_base(i,j) = 0;
%         end
%     end
% end
% %%% cal com1 ~ com6 %%%%%%%%%%
% d1 = [th1-pi/2 0 0]; % zyx: from frame{0} to COM1
% d2 = [th2-pi/2 -pi/2 0];
% d3 = [th3-pi/2 0 pi/2];
% d4 = [th4-pi/2 0 pi/2];
% d5 = [th5-pi/2 -pi/2 0];
% d6 = [th6-pi/2 0 pi/2];
% t1 = simplify(ET(d1));
% t2 = simplify(ET(d2));
% t3 = simplify(ET(d3));
% t4 = simplify(ET(d4));
% t5 = simplify(ET(d5));
% t6 = simplify(ET(d6));
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rotm1 = [  s1,  c1, 0;
%           -c1,  s1, 0;
%             0,   0, 1];
% rotm2 = [ 0,  c2, -s2;
%           0,  s2,  c2;
%           1,   0,  0];
% rotm3 = [  s3, 0, -c3;
%           -c3, 0, -s3;
%             0, 1,   0];
% rotm4 = [  s4, 0, -c4;
%           -c4, 0,  s4;
%             0, 1,   0];
% rotm5 = [ 0,  c5, -s5;
%           0,  s5,  c5;
%           1,   0    0];
% rotm6 = [  s6, 0, -c6;
%           -c6, 0, -s6;
%             0, 1,   0];
% % Here, xyz means relative position of COMi, which is relative to joint i
% % First, translation. Second, rotation.
% 
% A11 = cat(2,cat(1,rotm1,zeros(1,3)),[x1;y1;z1;1]);
% A21 = cat(2,cat(1,rotm2,zeros(1,3)),[x2;y2;z2;1]);
% A31 = cat(2,cat(1,rotm3,zeros(1,3)),[x3;y3;z3;1]);
% A41 = cat(2,cat(1,rotm4,zeros(1,3)),[x4;y4;z4;1]);
% A51 = cat(2,cat(1,rotm5,zeros(1,3)),[x5;y5;z5;1]);
% A61 = cat(2,cat(1,rotm6,zeros(1,3)),[x6;y6;z6;1]);
% 
% % used for dynamics 
% A_base = cat(2,cat(1,rotm_base,zeros(1,3)),[0;-L1;-L2;1]);
% A_tool = cat(2,cat(1,rotm_tool,zeros(1,3)),[0;0;0;1]);
% 
% T = A_base*(A*A_tool);
% T0 = A_base; %%　Tb0 --> base to motor1(x0,y0,z0)
% T1 = A_base*A1;
% T2 = A_base*(A1*A2);
% T3 = A_base*(A1*(A2*A3));
% T4 = A_base*(A1*(A2*(A3*A4)));
% T5 = A_base*(A1*(A2*(A3*(A4*A5)))); 
% T6 = A_base*(A1*(A2*(A3*(A4*(A5*A6))))); %% Tbe --> base to endpoint
% 
% Tc1 = A_base*A11; %% base to COM1
% Tc2 = A_base*(A1*A21);
% Tc3 = A_base*(A1*(A2*A31));
% Tc4 = A_base*(A1*(A2*(A3*A41)));
% Tc5 = A_base*(A1*(A2*(A3*(A4*A51))));
% Tc6 = A_base*(A1*(A2*(A3*(A4*(A5*A61)))));