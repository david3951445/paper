classdef EXE
    %Used to control how programs are executed
    % The purpose is to save your time. If some parts are okay, no need to
    % execute them. Or, you want to test some parts, then disable others.
    %
    % ex.
    %   F = 1   - execute f()
    %   F = 0   - load output of f() and not execute f()
    
    properties (Constant)
        A_B     = 0 % whether run linearize part
        LMI     = 0 % whether run LMI part
        TRAJ    = 0
        PLOT    = 1
    end
end