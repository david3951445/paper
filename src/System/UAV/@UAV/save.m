function Save(uav, whichVar)
%Save a property into UAV_FZmodel.mat
% whichVar  : which property be saved

switch whichVar
    case 'A'
        A = uav.A;
    case 'B'
        B = uav.B;
    case 'AB'
        AB = uav.AB;
    case 'K'
        K = uav.K;
    case 'tr'
        tr = uav.tr;
    otherwise
        disp(['No such property in UAV'])
end

if isfile([uav.PATH '.mat'])
    save(uav.PATH, whichVar, '-append');
else
    save(uav.PATH, whichVar);
end
    
end