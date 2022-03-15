%Methods for signal estimation
% The signal is assumed to be unknown, but the current and past value is knowned
% Thus, we can use an autoregressive model to describe the siganl
% Then, a signal is estimated.
% These methods only make sense if the siganl has some degree of autocorrelation.

clc; clear; close all
rng(0);
n = 1;
n_pre = n+5; % Window, determine how many previous time values were taken.
dt = 0.1;
t = 0 : dt : 50;

w = randn(1, length(t));
y = 3*cos(t) + 0*w - 0.1*t + 0.01*t.^2; % signal to be estimated

y_withInit = [zeros(1, n_pre), y];
% sys = ar(y, n);

for i = 1 : length(t)
    i_y_withInit = i + n_pre;
    y_cur = y_withInit(i_y_withInit-n_pre : i_y_withInit-1);
    y_cur2 = y_withInit(i_y_withInit-1 :-1: i_y_withInit-n);
    %% Base on AR model, using least square solution to obtain coeff.
    % method 1, MATLAB ar()
%     sys = ar(y_cur, n);
%     coeff = -sys.A(2 : n+1)';
    % method 2, my ar()
    coeff = my_ar(y_cur, n);
    
    % check the error
%     y_withInit(i_y_withInit-n-1 : i_y_withInit-1-1)*coeff - y_withInit(i_y_withInit-1)

    yh{1}(i) = y_cur2*coeff;
    
    %% Base on smooth model, coeff is a normalized geometric sequence.
    coeff2 = 0.98.^(0:n-1)'; coeff2 = coeff2/sum(coeff2);
    yh{2}(i) = y_cur2*coeff2;
    
    %% polyfit
    % method 1, fit()
%     f = fit(t(1:n_pre)', y_cur', 'poly2'); % This function can fit not only poly, but slower.
%     d = differentiate(f, t(n_pre));
    
    % method 2, polyfit()
    p = polyfit(t(1:n_pre), y_cur, 2);   
    d = polyval(polyder(p), t(n_pre));
    yh{3}(i) = y_withInit(i_y_withInit-1) + dt*d;
    
%     yh{3}(i) = polyval(p, t(n_pre+1));
         
    %% Taylor Expansion
    c_d = [2 -3 1];
    d = 0;
    for j = 1 : length(c_d)
        d = d + (c_d(j)*y_withInit(i_y_withInit-j))/dt;
    end
    yh{4}(i) = y_withInit(i_y_withInit-1) + dt*d;
    
%     log_coeff(:, i) = coeff;
end

Title = {'AR model', 'smooth model', 'polynomial fitting', 'Taylor expansion'};
for i = 1 : 4
e = y - yh{i};
subplot(4, 1, i)
plot(t, e, t, y);
title([Title{i}, ', err = ' num2str(norm(e))])
end