classdef EXE
    %Used to control how programs are executed
    % The purpose is to save your time. If some parts are okay, no need to
    % execute them. Or, you want to test some parts, then disable others.
    %
    % ex.
    %   SET_A = 1   - execute rb.set_A()
    %   SET_A = 0   - load rb.A and not execute set_A()
    
    properties (Constant)
        A       = 0
        B       = 0
        A_B     = 0
        LMI     = 1
        TRAJ    = 0
        PLOT    = 0
    end
end