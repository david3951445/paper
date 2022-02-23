function uav = getAB(uav)
%Construct AB
% parameter vector in lpv model : p = [x7 x9 x11]

% m = o.m, l = 0.2, b = 2, d = 5, G = 9.81
% Jx = 1.25, Jy = 1.25, Jz = 2.2
% Kx =o.Kx, Ky = o.Ky, Kz = o.Kz
% Kph = 0.012, Kth = 0.012, Kps = 0.012

% x7 = p(1);
% x9 = p(2);
% x11 = p(3);

% c7 = cos(x7);
% c9 = cos(x9);
% c11 = cos(x11);

% s7 = sin(x7);
% s9 = sin(x9);
% s11 = sin(x11);

% Note: This matrix is associated with uav.f() nad uav.g()
uav.ABl.lpv = cell(uav.DIM_X, uav.DIM_X + uav.DIM_U);
uav.ABl.lpv(:) = {@(p)0};
% A
uav.ABl.lpv{1, 2}   = @(p)1;
uav.ABl.lpv{2, 2}   = @(p)-uav.Kx/uav.m;
uav.ABl.lpv{3, 4}   = @(p)1;
uav.ABl.lpv{4, 4}   = @(p)-uav.Ky/uav.m;
uav.ABl.lpv{5, 6}   = @(p)1;
uav.ABl.lpv{6, 6}   = @(p)-uav.Kz/uav.m + uav.G;
uav.ABl.lpv{7, 8}   = @(p)1;
uav.ABl.lpv{8, 8}   = @(p)-uav.Kph/uav.Jx;
uav.ABl.lpv{9, 10}  = @(p)1;
uav.ABl.lpv{10, 10} = @(p)-uav.Kth/uav.Jy;
uav.ABl.lpv{11, 12} = @(p)1;
uav.ABl.lpv{12, 12} = @(p)-uav.Kps/uav.Jz;
% B
uav.ABl.lpv{2, 13}  = @(p)(cos(p(1))*sin(p(2))*cos(p(3)) + sin(p(1))*sin(p(3)))/uav.m;
uav.ABl.lpv{4, 13}  = @(p)(cos(p(1))*sin(p(2))*cos(p(3)) - sin(p(1))*cos(p(3)))/uav.m;
uav.ABl.lpv{6, 13}  = @(p)cos(p(1))*cos(p(2))/uav.m;
uav.ABl.lpv{8, 14}  = @(p)1/uav.Jx;
uav.ABl.lpv{10, 15} = @(p)1/uav.Jy;
uav.ABl.lpv{12, 16} = @(p)1/uav.Jz;
 
uav.ABl.num_p = length(uav.ABl.gridsize); % length of parameter vector of lpv system

uav.ABl.dep = zeros([size(uav.ABl.lpv) uav.ABl.num_p]);
% B
uav.ABl.dep(2, 13, :) = [1 1 1];
uav.ABl.dep(4, 13, :) = [1 1 1];
uav.ABl.dep(6, 13, :) = [1 1 0];

end